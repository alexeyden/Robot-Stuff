#ifndef FETCHER_H
#define FETCHER_H

#include <thread>
#include <mutex>
#include <atomic>

#include "crbuffer.h"
#include "vrep_client.h"

#include <glm/glm.hpp>

struct usonic_point {
    static constexpr float angle = usonic_msr_t::angle;
    glm::vec3 pos;
    glm::vec3 dir;
};

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

        T& value() {
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

    auto& laser_points() {
        return _laser_points;
    }

    typedef resource<crbuffer<usonic_point, 1024>> usonic_resource_t;
    typedef resource<crbuffer<glm::vec3, 64 * 64 * 2>> laser_resource_t;

private:
    void process_usonic(const usonic_msr_t& msr);
    void process_scam(const vrep_client::image_msr_scam_t& msr_left,
                      const vrep_client::image_msr_scam_t& msr_right);
    void process_laser(const vrep_client::image_msr_laser_t& msr);

    float _update_time;

    std::atomic<bool> _running;

    resource<crbuffer<usonic_point, 1024>> _usonic_points;
    resource<crbuffer<glm::vec3, 64 * 64 * 2>> _laser_points;

    resource<vrep_client::image_msr_laser_t> _laser_img;
    resource<vrep_client::image_msr_scam_t[2]> _scam_img;

    vrep_client _client;
    std::thread _thread;
};

#endif // FETCHER_H
