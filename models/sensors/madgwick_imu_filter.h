#pragma once

#include "grad.h"
#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

class MadgwickIMUFilter {

public:
    MadgwickIMUFilter();
    MadgwickIMUFilter(const Quaternion& q,
        float beta);
    MadgwickIMUFilter(Quaternion&& q,
        float beta);

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