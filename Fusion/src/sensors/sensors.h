#ifndef FETCHER_H
#define FETCHER_H

#include "pch.h"

#include "ffld/Detector.hpp"

#include "crbuffer.h"
#include "vrep_client.h"
#include "point_cloud.h"

struct scam_object {
    glm::vec3 pos;
    glm::vec3 size;
    glm::vec3 dir;

    bool near(const scam_object& o) {
        // TODO: obb intersection
        return glm::length(pos - o.pos) < 3;
    }
};

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
    typedef resource<std::vector<scam_object>> scam_objects_res_t;

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

    scam_objects_res_t& objects() {
        return _scam_objects;
    }

    bool pause;
private:
    void process_usonic(const usonic_msr_t& msr);
    void process_scam(vrep_client::image_msr_scam_t& msr_left,
                      vrep_client::image_msr_scam_t& msr_right);
    void process_laser(const vrep_client::image_msr_laser_t& msr);

    float _update_time;

    std::atomic<bool> _running;

    laser_img_res_t _laser_img;
    scam_img_res_t _scam_img;
    cloud_res_t _cloud;
    scam_objects_res_t _scam_objects;

    std::shared_ptr<FFLD::FFLDDetector> _detector;

    vrep_client _client;
    std::thread _thread;
};

#endif // FETCHER_H
