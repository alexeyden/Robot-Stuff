#include "vrep_client.h"

#include "v_repConst.h"

vrep_client::vrep_client() :
    _conn_id(-1),
    _lcam_id(-1), _rcam_id(-1),
    _lidar_cam_id(-1),
    _usonic_id {-1, -1, -1, -1}
{
}

vrep_client::~vrep_client()
{
    disconnect();
}

bool vrep_client::connect(simxInt port)
{
    int id = simxStart("127.0.0.1", port, 0, 1, 1000, 5);

    if(id >= 0) {
        simxGetObjectHandle(id, "CameraL", &_lcam_id, simx_opmode_blocking);
        simxGetObjectHandle(id, "CameraR", &_rcam_id, simx_opmode_blocking);

        simxGetObjectHandle(id, "Lidar", &_lidar_cam_id, simx_opmode_blocking);

        simxGetObjectHandle(id, "Sonic1", &_usonic_id[0], simx_opmode_blocking);
        simxGetObjectHandle(id, "Sonic2", &_usonic_id[1], simx_opmode_blocking);
        simxGetObjectHandle(id, "Sonic3", &_usonic_id[2], simx_opmode_blocking);
        simxGetObjectHandle(id, "Sonic4", &_usonic_id[3], simx_opmode_blocking);
    }

    _conn_id = id;

    return id >= 0;
}

void vrep_client::disconnect()
{
    if(is_connected())
        simxFinish(_conn_id);
}

bool vrep_client::is_connected() const
{
    return _conn_id != -1;
}

bool vrep_client::update_usonic(usonic_msr_t* msr)
{
    bool any_ok = true;
    bool op_ok = false;

    for(size_t i = 0; i < USONIC_NUM; i++) {
        simxFloat pos[3];
        simxFloat point[3];
        simxFloat angles[3];
        simxUChar state = 0;

        op_ok =
            simxGetObjectOrientation(_conn_id, _usonic_id[i], -1, angles, simx_opmode_blocking) == simx_return_ok &&
            simxGetObjectPosition(_conn_id, _usonic_id[i], -1, pos, simx_opmode_blocking) == simx_return_ok &&
            simxReadProximitySensor(_conn_id, _usonic_id[i], &state, point, nullptr, nullptr, simx_opmode_blocking) == simx_return_ok;

        if(op_ok) {
            glm::mat4 rot = glm::rotate(glm::mat4(1.0f), angles[0], glm::vec3(1.0f, 0.0f, 0.0f)) *
                glm::rotate(glm::mat4(1.0f), angles[1], glm::vec3(0.0f, 1.0f, 0.0f)) *
                glm::rotate(glm::mat4(1.0f), angles[2], glm::vec3(0.0f, 0.0f, 1.0f));

            auto ps = glm::vec4(pos[0], pos[1], pos[2], 1.0f);
            float dist = glm::length(glm::vec3(point[0], point[1], point[2]));

            if(state == 0)
                dist = 3.0f;

            msr[i].pos0 = ps;
            msr[i].pos1 = rot * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f) * dist + ps;
            msr[i].maxed = state == 0;
        }

        any_ok &= op_ok;
    }

    return any_ok;

}

bool vrep_client::update_laser(vrep_client::image_msr_laser_t &msr)
{
    float angles[3];
    simxInt resolution[2];

    simxFloat* image;
    simxFloat pos[3];

    bool op_ok = false;

    op_ok =
        simxGetObjectOrientation(_conn_id, _lidar_cam_id, -1, angles, simx_opmode_blocking) == simx_return_ok &&
        simxGetObjectPosition(_conn_id, _lidar_cam_id, -1, pos, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorDepthBuffer(_conn_id, _lidar_cam_id, resolution, &image, simx_opmode_blocking) == simx_return_ok;

    if(op_ok) {
        glm::mat4 rot = glm::rotate(glm::mat4(1.0f), angles[0], glm::vec3(1.0f, 0.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), angles[1], glm::vec3(0.0f, 1.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), angles[2], glm::vec3(0.0f, 0.0f, 1.0f));
        msr.dir = rot * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
        msr.up = rot * glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
        msr.dir = glm::normalize(msr.dir);
        msr.up = glm::normalize(msr.up);
        msr.pos = glm::vec3(pos[0], pos[1], pos[2]);
        memcpy(msr.buf, image, resolution[0] * resolution[1] * sizeof(float));
    }

    return op_ok;

}

bool vrep_client::update_scam(vrep_client::image_msr_scam_t &lmsr, vrep_client::image_msr_scam_t &rmsr)
{
    float angles[3];
    simxInt resolution[2];

    simxUChar *image_left, *image_right;
    simxFloat pos_left[3], pos_right[3];

    bool op_ok = false;

    op_ok =
        simxGetObjectOrientation(_conn_id, _lcam_id, -1, &angles[0], simx_opmode_blocking) == simx_return_ok &&
        simxGetObjectPosition(_conn_id, _lcam_id, -1, (float*) &pos_left, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorImage(_conn_id, _lcam_id, &resolution[0], &image_left, 1, simx_opmode_blocking) == simx_return_ok &&

        simxGetObjectPosition(_conn_id, _rcam_id, -1, (float*) &pos_right, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorImage(_conn_id, _rcam_id, &resolution[0], &image_right, 1, simx_opmode_blocking) == simx_return_ok;

    if(op_ok) {
        glm::mat4 rot = glm::rotate(glm::mat4(1.0f), angles[0], glm::vec3(1.0f, 0.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), angles[1], glm::vec3(0.0f, 1.0f, 0.0f)) *
            glm::rotate(glm::mat4(1.0f), angles[2], glm::vec3(0.0f, 0.0f, 1.0f));
        auto dir = rot * glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        auto up = rot * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);

        lmsr.dir = dir;
        lmsr.up = up;
        lmsr.pos = glm::vec3(pos_left[0], pos_left[1], pos_left[2]);
        memcpy(lmsr.buf, image_left, resolution[0] * resolution[1] * sizeof(uint8_t));

        lmsr.dir = dir;
        lmsr.up = up;
        lmsr.pos = glm::vec3(pos_right[0], pos_right[1], pos_right[2]);
        memcpy(rmsr.buf, image_right, resolution[0] * resolution[1] * sizeof(uint8_t));
    }

    return op_ok;
}
