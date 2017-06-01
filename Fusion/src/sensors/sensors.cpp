#include "sensors.h"

sensors::sensors():
    _update_time(0.0f), _running(false)
{
    pause = false;
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

                self->_laser_img.mutex().lock();
                new_laser = self->_client.update_laser(self->_laser_img.value());
                self->_laser_img.mutex().unlock();

                self->_scam_img.mutex().lock();
                self->_client.update_scam(self->_scam_img._value[0], self->_scam_img._value[1]);
                self->_scam_img.mutex().unlock();

                usonic_msr_t msr[vrep_client::USONIC_NUM];
                new_usonic = self->_client.update_usonic(msr);

                if((new_usonic || new_laser) && !self->pause) {
                    self->_cloud.mutex().lock();
                    if(new_laser)
                        self->process_laser(self->_laser_img._value);
                    if(new_usonic)
                        for(size_t i = 0; i < vrep_client::USONIC_NUM; i++)
                            self->process_usonic(msr[i]);
                    self->_cloud.mutex().unlock();
                }

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

void sensors::process_scam(const vrep_client::image_msr_scam_t &msr_left, const vrep_client::image_msr_scam_t &msr_right)
{
    (void) msr_left;
    (void) msr_right;
    //teehee
}

void sensors::process_laser(const vrep_client::image_msr_laser_t &msr)
{
    const float fov = glm::radians(60.0f);
    const float near = 0.1f;
    const float far = 10.0f;

    glm::mat4 model = glm::lookAtRH(msr.pos, msr.pos - msr.dir, msr.up);

    for(size_t x = 0; x < msr.WIDTH; x++)
        for(size_t y = 0; y < msr.HEIGHT; y++) {
            float d = msr.buf[x + y * msr.WIDTH];
            if(d > 0.9f)
                continue;

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
