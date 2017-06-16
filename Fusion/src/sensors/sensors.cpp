#include "sensors.h"

using namespace FFLD;

sensors::sensors():
    _update_time(0.0f), _running(false)
{
    pause = false;

    _detector_car = std::shared_ptr<FFLDDetector> { new FFLDDetector("ffld/models/car_2d.txt") };
    _detector_car->threshold = -0.5f;
    _detector_car->interval = 4;

    _detector_man = std::shared_ptr<FFLDDetector> { new FFLDDetector("ffld/models/person_2d.txt") };
    _detector_man->threshold = -0.5f;
    _detector_man->interval = 4;
}

sensors::~sensors()
{
    _laser_img.mutex().unlock();
    _scam_img.mutex().unlock();
    _cloud.mutex().unlock();

    _running = false;

    if(_thread.joinable())
        _thread.join();
}

void sensors::start()
{
    auto thread_fn = [](sensors* self) {
        self->_client.connect(19997);

        while(self->_running == true) {
            if(self->_client.is_connected()) {
                auto start = std::chrono::system_clock::now();
                bool new_usonic = false;
                bool new_laser = false;
                bool new_scam = false;

                self->_laser_img.mutex().lock();
                new_laser = self->_client.update_laser(self->_laser_img.value());
                self->_laser_img.mutex().unlock();

                self->_scam_img.mutex().lock();
                new_scam = self->_client.update_scam(self->_scam_img._value[0], self->_scam_img._value[1]);
                if(new_scam)
                    self->process_scam(self->_scam_img._value[0], self->_scam_img._value[1]);
                self->_scam_img.mutex().unlock();

                usonic_msr_t msr[vrep_client::USONIC_NUM];
                new_usonic = self->_client.update_usonic(msr);

                if((new_usonic || new_laser || new_scam) && !self->pause) {
                    self->_cloud.mutex().lock();
                    if(new_laser)
                        self->process_laser(self->_laser_img._value);
                    if(new_usonic)
                        for(size_t i = 0; i < vrep_client::USONIC_NUM; i++)
                            self->process_usonic(msr[i]);
                    self->_cloud.mutex().unlock();
                }

                self->_robot = self->_client.update_robot();
                self->robot.store(&self->_robot);

                auto end = std::chrono::system_clock::now();

                std::chrono::duration<float> dur = end - start;
                self->_update_time = dur.count();
            }
        }
    };

    _running = true;
    _thread = std::thread(thread_fn, this);
}

void sensors::stop()
{
    _running = false;
}

void sensors::process_usonic(const usonic_msr_t &msr)
{
    _cloud._value.add_usonic(msr.pos0, msr.pos1, msr.angle, msr.maxed);
}

void sensors::process_scam(vrep_client::image_msr_scam_t &msr_left, vrep_client::image_msr_scam_t &msr_right)
{
    (void) msr_right;

    //TODO: disparity map

    std::vector<scam_object> cars = detect(_detector_car, msr_left,
                                           object_descr {
                                               .size = {4.8f, 2.2f, 1.5f},
                                               .dir = {0, glm::pi<float>()/2.0f}
                                           });
    std::vector<scam_object> ppl = detect(_detector_man, msr_left,
                                           object_descr {
                                               .size = {0.5, 0.26, 2.0f},
                                               .dir = {glm::pi<float>()/2.0f, 0}
                                           });
    cars.insert(cars.begin(), ppl.begin(), ppl.end());

    // TODO: track locations and update pos. based on prob. and nearby points
    _scam_objects.mutex().lock();
    for(scam_object& o : cars) {
        auto iter = std::find_if(_scam_objects.value().begin(), _scam_objects.value().end(),
                                 [&o](const scam_object& v) { return v.near(o); });
        if(iter != _scam_objects.value().end()) *iter = o;
        else _scam_objects.value().push_back(o);
    }
    _scam_objects.mutex().unlock();
}

void sensors::process_laser(const vrep_client::image_msr_laser_t &msr)
{
    // TODO: fetch these from camera props on connect
    const float fov = glm::radians(60.0f);
    const float near = 0.1f;
    const float far = 10.0f;

    glm::mat4 model = glm::lookAtRH(msr.pos, msr.pos - msr.dir, msr.up);

    for(size_t x = 0; x < msr.WIDTH; x++)
        for(size_t y = 0; y < msr.HEIGHT; y++) {
            float d = msr.buf[x + y * msr.WIDTH];
            if(d > 0.9f)
                continue;

            // stolen from vrep sources
            // calculates inv(projection) * inv(bias) * (x,y,d,1)

            float xAlpha=0.5f/(std::tan(fov*0.5f));
            float yAlpha=0.5f/(std::tan(fov*0.5f));

            float xBeta=2.0f*std::tan(fov*0.5f);
            float yBeta=2.0f*std::tan(fov*0.5f);

            float tanYDistTyAlpha=std::tan(-fov/2.0f + fov * y/63.0f)*yAlpha;
            float tanXDistTxAlpha=std::tan(-fov/2.0f + fov * (63 - x)/63.0f)*xAlpha;

            float zDist=near+d*(far - near) + (glm::linearRand(0.0f, 1.0f) < 0.001 ? glm::linearRand(0.1f, 10.0f) : 0.0f);
            glm::vec3 rnd = glm::gaussRand(glm::vec3(
                                               tanXDistTxAlpha*xBeta*zDist,
                                               tanYDistTyAlpha*yBeta*zDist, zDist),
                                           glm::vec3(0.1f, 0.1f, 0.1f));
            glm::vec4 v = glm::vec4(rnd, 1.0f);

            glm::vec3 point = glm::inverse(model) * v;
            _cloud._value.add_laser(msr.pos, point);
        }
}

std::vector<scam_object> sensors::detect(std::shared_ptr<FFLDDetector> detector, vrep_client::image_msr_scam_t &msr, const sensors::object_descr &descr)
{
    const size_t IW = msr.WIDTH;
    const size_t IH = msr.HEIGHT;
    static uint8_t tmp[IW*IH];

    std::vector<scam_object> objs;

    // flip y
    for(int i = 0; i < IH; i++)
        memcpy(tmp + i * IW, msr.buf + (IH - i - 1)*IW, IW);
    std::vector<Detection> detections = detector->run(msr.WIDTH, msr.HEIGHT, 1, tmp);

    if(detections.size() == 0)
        return objs;

    const int max = std::numeric_limits<int>::max();
    const int min = std::numeric_limits<int>::min();

    int x0 = max, y0 = max, x1 = min, y1 = min;

    //TODO: check for intersection, otherwise handle as multiple objects
    // unite bboxes if many
    for(const Detection& d : detections) {
        x0 = std::min(d.left(), x0);
        y0 = std::min(d.top(), y0);
        x1 = std::max(d.right(), x1);
        y1 = std::max(d.bottom(), y1);
    }
    bool cropped = x0 == 0 || x1 == msr.WIDTH-1;

    // draw bboxes
    auto border =cropped ? [](int t) { return (t%2) * 0xff; } : [](int) { return 0x00; };
    for(int x = x0; x <= x1; x++) {
        msr.set_pixel(x, IH-1-y0, border(x));
        msr.set_pixel(x, IH-1-y1, border(x));
        msr.set_pixel(x, IH-1-y0, border(x));
        msr.set_pixel(x, IH-1-y1, border(x));
    }

    for(int y = y0; y <= y1; y++) {
        msr.set_pixel(x0, IH-1-y, border(y));
        msr.set_pixel(x1, IH-1-y, border(y));
        msr.set_pixel(x0, IH-1-y, border(y));
        msr.set_pixel(x1, IH-1-y, border(y));
    }

    //TODO: return predicted bbox from detector
    // for now, exit if bbox appears to be cropped
    if(cropped)
        return objs;

    // TODO: tag view clusters in mixture with orientations and sizes
    const bool from_side = std::abs(msr.dir.x) > std::abs(msr.dir.y)  ;

    const float near = 0.01f;
    const float far = 100.0f;
    const float fov = glm::radians(60.0f);

    float size = from_side ? descr.size.y : descr.size.x;
    float size2 = from_side ? descr.size.x : descr.size.y;
    float obj_dir = from_side ? descr.dir.y : descr.dir.x;

    std::cerr << "Detector: from side = " << from_side << std::endl;
    std::cerr << "Detector: dim frontal = " << size << " dim lateral = " << size2 << std::endl;

    float xAlpha=0.5f/(std::tan(fov*0.5f));
    float xBeta=2.0f*std::tan(fov*0.5f);

    float tanXDistTxAlpha0=std::tan(-fov/2.0f + fov * (float(x0)/msr.WIDTH))*xAlpha;
    float tanXDistTxAlpha1=std::tan(-fov/2.0f + fov * (float(x1)/msr.WIDTH))*xAlpha;

    const float px0 = tanXDistTxAlpha0*xBeta*near;
    const float px1 = tanXDistTxAlpha1*xBeta*near;

    const float depth = near * size / std::abs(px0 - px1) + size2; // /2;

    glm::vec4 pv = glm::vec4((tanXDistTxAlpha0 + tanXDistTxAlpha1)/2.0f*xBeta*depth, 0, -depth, 1);

    glm::mat4 model = glm::lookAt(msr.pos, msr.pos + msr.dir, msr.up);
    glm::vec4 p = glm::inverse(model) * pv;

    const float angle = +std::atan2(msr.dir.y, msr.dir.x) + obj_dir;
    scam_object o {
        .pos = p,
        .size = descr.size,
        .dir = glm::vec3(std::cos(angle), std::sin(angle), 0)
    };

    objs.push_back(o);

    return objs;
}
