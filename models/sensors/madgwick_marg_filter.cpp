#include "madgwick_marg_filter.h"
#include "matrix.h"
#include "vector4f.h"
#include <cmath>

namespace FrameDrag {

MadgwickMARGFilter::MadgwickMARGFilter()
    : _current_estimate{ 0.0f, 0.0f, 0.0f, 0.0f }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f }
    , _beta{ 0.0f }
    , _bias_convergence_rate{ 0.0f }
{
}

MadgwickMARGFilter::MadgwickMARGFilter(const Quaternion& q,
    float beta,
    float bias_convergence_rate)
    : _current_estimate{ q }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f }
    , _beta{ beta }
    , _bias_convergence_rate{ bias_convergence_rate }
{
}

MadgwickMARGFilter::MadgwickMARGFilter(Quaternion&& q, float beta,
    float bias_convergence_rate)
    : _current_estimate{ std::move(q) }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f, 0.0f }
    , _beta{ beta }
    , _bias_convergence_rate{ bias_convergence_rate }
{
}

void MadgwickMARGFilter::update(const Vector3f& gyro_m,
    const Vector3f& accel_m,
    const Vector3f& magnet_m,
    float time_delta)
{
    Quaternion current_estimate_derivative = 0.5 * _current_estimate * Quaternion(0.0f, gyro_m - _estimated_bias);

    auto gravity_grad = grad(_current_estimate,
        { 0.0f, 0.0f, -9.8f },
        accel_m);

    auto bfield_world_frame = _current_estimate.apply(magnet_m);
    bfield_world_frame[0] = std::sqrt(std::pow(bfield_world_frame[0], 2) + std::pow(bfield_world_frame[1], 2));
    bfield_world_frame[1] = 0.0f;

    auto bfield_grad = grad(_current_estimate,
        bfield_world_frame,
        magnet_m);

    auto total_grad = gravity_grad + bfield_grad;
    auto normalised_total_grad = total_grad / total_grad.norm();

    auto err = 2.0f * _current_estimate.conjugate() * normalised_total_grad;
    _estimated_bias += _bias_convergence_rate * err.im() * time_delta;

    _current_estimate += (current_estimate_derivative - _beta * normalised_total_grad) * time_delta;
    _current_estimate /= _current_estimate.norm();
}

Quaternion& MadgwickMARGFilter::estimate()
{
    return _current_estimate;
}
}