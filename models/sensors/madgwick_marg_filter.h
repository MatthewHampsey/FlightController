#pragma once

#include "grad.h"
#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

class MadgwickMARGFilter {

public:
    MadgwickMARGFilter();
    MadgwickMARGFilter(const Quaternion& q, float beta,
        float bias_convergence_rate);
    MadgwickMARGFilter(Quaternion&& q, float beta, float bias_convergence_rate);

    void update(const Vector3f& gyro_m, const Vector3f& accel_m,
        const Vector3f& magnet_m, float time_delta);

    Quaternion& estimate();

private:
    Quaternion _current_estimate;
    Vector3f _estimated_bias;
    float _beta;
    float _bias_convergence_rate;
};
}