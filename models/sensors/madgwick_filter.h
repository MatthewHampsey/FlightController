#pragma once

#include "quaternion.h"

namespace FrameDrag {

Quaternion grad(const Quaternion& sensor_frame_to_world,
    const Vector3f& v,
    const Vector3f& measurement);

class MadgwickFilter {

public:
    MadgwickFilter();
    MadgwickFilter(const Quaternion& q, float beta);
    MadgwickFilter(Quaternion&& q, float beta);

    void update(const Vector3f& gyro_m,
        const Vector3f& accel_m,
        const Vector3f& magnet_m,
        float time_delta);
    void update(const Vector3f& gyro_m,
        const Vector3f& accel_m,
        float time_delta);

    Quaternion& estimate();
    void setBeta(float beta);

private:
    Quaternion _current_estimate;
    float _beta;
};
}