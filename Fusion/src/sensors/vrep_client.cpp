#include "vrep_client.h"
#include "v_repConst.h"

#include <cstring>
#include <cmath>

vrep_client::vrep_client() :
    _conn_id(-1),
    _lcam_id(-1), _rcam_id(-1),
    _lidar_cam_id(-1),
    _usonic_id {-1, -1, -1}
{
}

vrep_client::~vrep_client()
{
    disconnect();
}

bool vrep_client::connect(simxInt port)
{
    _conn_id = simxStart("127.0.0.1", port, 0, 1, 1000, 5);

    if(is_connected()) {
        simxGetObjectHandle(_conn_id, "CameraL", &_lcam_id, simx_opmode_blocking);
        simxGetObjectHandle(_conn_id, "CameraR", &_rcam_id, simx_opmode_blocking);

        simxGetObjectHandle(_conn_id, "Lidar", &_lidar_cam_id, simx_opmode_blocking);

        simxGetObjectHandle(_conn_id, "Sonic1", &_usonic_id[0], simx_opmode_blocking);
        simxGetObjectHandle(_conn_id, "Sonic2", &_usonic_id[1], simx_opmode_blocking);
        simxGetObjectHandle(_conn_id, "Sonic3", &_usonic_id[2], simx_opmode_blocking);
    }

    return is_connected();
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

bool vrep_client::update_usonic(usonic_msr_t &msr)
{
    bool any_ok = false;
    bool op_ok = false;

    for(size_t i = 0; i < USONIC_NUM; i++) {
        float angles[3];
        vec3_t pos;
        simxFloat point[3];
        simxUChar state;

        op_ok =
            simxGetObjectOrientation(_conn_id, _usonic_id[i], -1, angles, simx_opmode_blocking) == simx_return_ok &&
            simxGetObjectPosition(_conn_id, _usonic_id[i], -1, (float*) &pos, simx_opmode_blocking) == simx_return_ok &&
            simxReadProximitySensor(_conn_id, _usonic_id[i], &state, point, nullptr, nullptr, simx_opmode_blocking);

        if(op_ok) {
            msr.dir = vec3_t { (float) cos(angles[0]), (float) sin(angles[1]), (float) sin(angles[2]) };
            msr.pos = pos;
            msr.dist = sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
        }

        any_ok |= op_ok;
    }

    return any_ok;

}

bool vrep_client::update_laser(vrep_client::image_msr_laser_t &msr)
{
    float angles[3];
    simxInt resolution[2];

    simxFloat* image;
    vec3_t pos;

    bool op_ok = false;

    op_ok =
        simxGetObjectOrientation(_conn_id, _lidar_cam_id, -1, angles, simx_opmode_blocking) == simx_return_ok &&
        simxGetObjectPosition(_conn_id, _lidar_cam_id, -1, (float*) &pos, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorDepthBuffer(_conn_id, _lidar_cam_id, resolution, &image, simx_opmode_blocking) == simx_return_ok;

    if(op_ok) {
        msr.dir = vec3_t { (float) cos(angles[0]), (float) sin(angles[1]), (float) sin(angles[2]) };
        msr.pos = pos;
        memcpy(msr.buf, image, resolution[0] * resolution[1] * sizeof(float));
    }

    return op_ok;

}

bool vrep_client::update_scam(vrep_client::image_msr_scam_t &lmsr, vrep_client::image_msr_scam_t &rmsr)
{
    float angles[3];
    simxInt resolution[2];

    simxUChar *image_left, *image_right;
    vec3_t pos_left, pos_right;

    bool op_ok = false;

    op_ok =
        simxGetObjectOrientation(_conn_id, _lcam_id, -1, &angles[0], simx_opmode_blocking) == simx_return_ok &&
        simxGetObjectPosition(_conn_id, _lcam_id, -1, (float*) &pos_left, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorImage(_conn_id, _lcam_id, &resolution[0], &image_left, 1, simx_opmode_blocking) == simx_return_ok &&

        simxGetObjectPosition(_conn_id, _rcam_id, -1, (float*) &pos_right, simx_opmode_blocking) == simx_return_ok &&
        simxGetVisionSensorImage(_conn_id, _rcam_id, &resolution[0], &image_right, 1, simx_opmode_blocking) == simx_return_ok;

    if(op_ok) {
        auto dir = vec3_t { (float) cos(angles[0]), (float) sin(angles[1]), (float) sin(angles[2]) };

        lmsr.dir = dir;
        lmsr.pos = pos_left;
        memcpy(lmsr.buf, image_left, resolution[0] * resolution[1] * sizeof(uint8_t));

        rmsr.dir = dir;
        rmsr.pos = pos_right;
        memcpy(rmsr.buf, image_right, resolution[0] * resolution[1] * sizeof(uint8_t));
    }

    return op_ok;
}
