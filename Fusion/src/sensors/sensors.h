#ifndef FETCHER_H
#define FETCHER_H

#include <thread>
#include <mutex>
#include <atomic>

#include <glm/glm.hpp>

#include "point_cloud.h"
#include "crbuffer.h"
#include "vrep_client.h"

class sensors
{
public:
    template<typename T>
    class resource {
    public:
        resource() {}

        const T& value() const {
            return _value;
        }

        T& value() {
            return _value;
        }

        std::mutex& mutex() {
            return _mutex;
        }

        friend class sensors;

    private:
        std::mutex _mutex;
        T _value;
    };

public:
    typedef resource<vrep_client::image_msr_laser_t> laser_img_res_t;
    typedef resource<vrep_client::image_msr_scam_t[2]> scam_img_res_t;
    typedef resource<point_cloud> cloud_res_t;

    sensors();
    ~sensors();

    void start();
    void stop();

    const vrep_client& client() const {
        return _client;
    }

    float update_time() const {
        return _update_time;
    }

    laser_img_res_t& laser_img() {
        return _laser_img;
    }

    scam_img_res_t& scam_img() {
        return _scam_img;
    }

    cloud_res_t& cloud() {
        return _cloud;
    }

    bool pause;
private:
    void process_usonic(const usonic_msr_t& msr);
    void process_scam(const vrep_client::image_msr_scam_t& msr_left,
                      const vrep_client::image_msr_scam_t& msr_right);
    void process_laser(const vrep_client::image_msr_laser_t& msr);

    float _update_time;

    std::atomic<bool> _running;

    laser_img_res_t _laser_img;
    scam_img_res_t _scam_img;
    cloud_res_t _cloud;

    vrep_client _client;
    std::thread _thread;
};

#endif // FETCHER_H
