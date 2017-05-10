#include "fetcher.h"
#include <chrono>

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
                {
                    std::lock_guard<std::mutex>(self->_laser_img.mutex());
                    if(self->_client.update_laser(self->_laser_img._value)) {
                        self->_laser_img._is_null = false;
                    }
                }
                {
                    std::lock_guard<std::mutex>(self->_scam_img.mutex());
                    if(self->_client.update_scam(self->_scam_img._value[0], self->_scam_img._value[1])) {
                        self->_scam_img._is_null = false;
                    }
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

void fetcher::stop()
{
    _running = false;
}
