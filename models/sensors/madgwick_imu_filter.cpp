#include "madgwick_imu_filter.h"
#include "matrix.h"
#include "vector4f.h"
#include <cmath>

namespace FrameDrag {

MadgwickIMUFilter::MadgwickIMUFilter()
    : _current_estimate{ 0.0f, 0.0f, 0.0f, 0.0f }
    , _beta{ 0.0f }
{
}

MadgwickIMUFilter::MadgwickIMUFilter(const Quaternion& q,
    float beta)
    : _current_estimate{ q }
    , _beta{ beta }
{
}

MadgwickIMUFilter::MadgwickIMUFilter(Quaternion&& q, float beta)
    : _current_estimate{ std::move(q) }
    , _beta{ beta }
{
}

void MadgwickIMUFilter::update(const Vector3f& gyro_m,
    const Vector3f& accel_m,
    float time_delta)
{
    Quaternion current_estimate_derivative = _current_estimate * Quaternion(0.0f, gyro_m);

    auto gravity_grad = grad(_current_estimate,
        { 0.0f, 0.0f, -9.8f },
        accel_m);
    auto total_grad = gravity_grad;
    auto normalised_total_grad = _beta * total_grad / total_grad.norm(); ///(total_grad.norm() + 0.00001f);

    _current_estimate += (current_estimate_derivative - normalised_total_grad) * time_delta;
    _current_estimate /= _current_estimate.norm();
}

Quaternion& MadgwickIMUFilter::estimate()
{
    return _current_estimate;
}

void MadgwickIMUFilter::setBeta(float beta)
{
    _beta = beta;
}
}