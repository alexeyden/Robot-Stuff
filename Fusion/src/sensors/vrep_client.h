#ifndef VREP_CLIENT_H
#define VREP_CLIENT_H

#include "pch.h"

#include "extApi.h"

struct usonic_msr_t {
    static constexpr float angle = M_PI * 30.0f / 180.0f;
    glm::vec3 pos0;
    glm::vec3 pos1;
    bool maxed;
};

template<typename T, size_t width, size_t height>
struct image_msr_t {
    static constexpr size_t WIDTH = width;
    static constexpr size_t HEIGHT = height;
    image_msr_t() {
        buf = new T[WIDTH * HEIGHT];
    }
    ~image_msr_t() {
        if(buf != nullptr)
            delete [] buf;
    }

    void set_pixel(size_t x, size_t y, T value) {
        buf[x + y * WIDTH] = value;
    }

    T* buf;
    glm::vec3 dir;
    glm::vec3 up;
    glm::vec3 pos;
};

class vrep_client
{
public:
    typedef image_msr_t<float, 64, 64> image_msr_laser_t;
    typedef image_msr_t<uint8_t, 256, 256> image_msr_scam_t;

    static constexpr size_t USONIC_NUM = 4;

public:
    vrep_client();
    ~vrep_client();

    vrep_client& operator=(const vrep_client& cp) {
        _conn_id = cp._conn_id;
        _lcam_id = cp._lcam_id;
        _rcam_id = cp._rcam_id;
        _lidar_cam_id = cp._lidar_cam_id;

        for(size_t i = 0; i < USONIC_NUM; i++)
            _usonic_id[i] = cp._usonic_id[i];

        return *this;
    }

    bool connect(simxInt port);
    void disconnect();

    bool is_connected() const;

    bool update_usonic(usonic_msr_t *msr);
    bool update_laser(vrep_client::image_msr_laser_t &msr);
    bool update_scam(vrep_client::image_msr_scam_t &lmsr, image_msr_scam_t &rmsr);
    std::pair<glm::vec3, glm::vec3> update_robot();

private:
    simxInt _conn_id;
    simxInt _robot_id;
    simxInt _lcam_id, _rcam_id, _lidar_cam_id;
    simxInt _usonic_id[USONIC_NUM];
};

#endif // VREP_CLIENT_H
