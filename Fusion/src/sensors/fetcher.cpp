#include "fetcher.h"

#include <iostream>
#include <chrono>

#include <glm/gtc/matrix_transform.hpp>

fetcher::fetcher():
    _update_time(0.0f), _running(false)
{
    _laser_img._value.buf = new float[_laser_img._value.WIDTH * _laser_img._value.HEIGHT];
}

fetcher::~fetcher()
{
    _laser_img.mutex().unlock();
    _scam_img.mutex().unlock();

    _running = false;

    if(_thread.joinable())
        _thread.join();
}

void fetcher::start()
{
    auto thread_fn = [](fetcher* self) {
        self->_client.connect(19997);

        while(self->_running == true) {
            if(self->_client.is_connected()) {
                auto start = std::chrono::system_clock::now();
                bool new_value = false;

                self->_laser_img.mutex().lock();
                if(self->_client.update_laser(self->_laser_img._value)) {
                    new_value = true;
                    self->_laser_img._is_null = false;
                }
                self->_laser_img.mutex().unlock();

                if(new_value) {
                    self->_laser_points.mutex().lock();
                    self->process_laser(self->_laser_img._value);
                    self->_laser_points.mutex().unlock();
                }

                new_value = false;

                self->_scam_img.mutex().lock();
                if(self->_client.update_scam(self->_scam_img._value[0], self->_scam_img._value[1])) {
                    new_value = true;
                    self->_scam_img._is_null = false;
                }
                self->_scam_img.mutex().unlock();

                auto end = std::chrono::system_clock::now();

                std::chrono::duration<float> dur = end - start;
                self->_update_time = dur.count();
            }
        }
    };

    _running = true;
    _thread = std::thread(thread_fn, this);
}

void fetcher::stop()
{
    _running = false;
}

void fetcher::process_usonic(const usonic_msr_t &msr)
{
    glm::vec3 wpos = msr.pos + msr.dist * msr.dir;
    _usonic_points._value.put(usonic_point { .pos = wpos, .dir = msr.dir });
}

void fetcher::process_scam(const vrep_client::image_msr_scam_t &msr_left, const vrep_client::image_msr_scam_t &msr_right)
{

}

void fetcher::process_laser(const vrep_client::image_msr_laser_t &msr)
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

            float zDist=near+d*(far - near);
            glm::vec4 v(tanXDistTxAlpha*xBeta*zDist,tanYDistTyAlpha*yBeta*zDist,zDist, 1.0f);

            glm::vec3 point = glm::inverse(model) * v;
            _laser_points._value.put(std::move(point));
        }
}
