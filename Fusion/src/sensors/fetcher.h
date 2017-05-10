#ifndef FETCHER_H
#define FETCHER_H

#include <thread>
#include <mutex>
#include <atomic>

#include "vrep_client.h"

class fetcher
{
public:
    template<typename T>
    class resource {
    public:
        resource() : _is_null(true) {}

        const T& value() const {
            return _value;
        }

        std::mutex& mutex() {
            return _mutex;
        }

        bool is_null() const {
            return _is_null;
        }

        friend class fetcher;
    private:
        std::mutex _mutex;
        T _value;
        bool _is_null;
    };

public:
    fetcher();
    ~fetcher();

    void start();
    void stop();

    const vrep_client& client() const {
        return _client;
    }

    float update_time() const {
        return _update_time;
    }

    resource<vrep_client::image_msr_laser_t>& laser_img()
    {
        return _laser_img;
    }

    resource<vrep_client::image_msr_scam_t[2]>& scam_img() {
        return _scam_img;
    }

private:
    float _update_time;

    std::atomic<bool> _running;

    resource<vrep_client::image_msr_laser_t> _laser_img;
    resource<vrep_client::image_msr_scam_t[2]> _scam_img;

    vrep_client _client;
    std::thread _thread;
};

#endif // FETCHER_H
