#include "mahony_complimentary_filter.h"
#include "matrix.h"
#include "vector4f.h"
#include <cmath>
#include <cassert>

namespace FrameDrag {

MahonyFilter::MahonyFilter()
    : _current_estimate{ 0.0f, 0.0f, 0.0f, 0.0f }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f }
    , _reference_directions{}
    , _k_p{ 0.0f }
    , _k_i{ 0.0f }
{
}

MahonyFilter::MahonyFilter(const Quaternion& q,
    std::vector<std::pair<unsigned int, Vector3f>> reference_directions,
    float k_p, float k_i)
    : _current_estimate{ q }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f }
    , _reference_directions{reference_directions}
    , _k_p{ k_p }
    , _k_i{ k_i }
    {}

MahonyFilter::MahonyFilter(Quaternion&& q,
    std::vector<std::pair<unsigned int, Vector3f>> reference_directions,
    float k_p, float k_i)
    : _current_estimate{ q }
    , _estimated_bias{ 0.0f, 0.0f, 0.0f }
    , _reference_directions{reference_directions}
    , _k_p{ k_p }
    , _k_i{ k_i }
    {}

void MahonyFilter::update(const Vector3f& gyro_m,
        std::vector<Vector3f> measurements,
        float time_delta)
{
    Vector3f correction_term{0.0f, 0.0f, 0.0f};
    assert(measurements.size() == _reference_directions.size());
    for(size_t i = 0; i < measurements.size(); i++){
        auto k = _reference_directions[i].first;
        auto ref_meas = _current_estimate.inverse().apply(_reference_directions[i].second);
        correction_term += k*measurements[i].cross(ref_meas);
    }
    Quaternion quat_deriv = 0.5f*_current_estimate*Quaternion(0.0f, 
                                        gyro_m - _estimated_bias + _k_p*correction_term);
    Vector3f bias_deriv = -_k_i*correction_term;

    _current_estimate += quat_deriv*time_delta;
    _current_estimate /= _current_estimate.norm();

    _estimated_bias += bias_deriv*time_delta;

    // Quaternion current_estimate_derivative = 0.5 * _current_estimate * Quaternion(0.0f, gyro_m - _estimated_bias);

    // auto gravity_grad = grad(_current_estimate,
    //     { 0.0f, 0.0f, -9.8f },
    //     accel_m);

    // auto bfield_world_frame = _current_estimate.apply(magnet_m);
    // bfield_world_frame[0] = std::sqrt(std::pow(bfield_world_frame[0], 2) + std::pow(bfield_world_frame[1], 2));
    // bfield_world_frame[1] = 0.0f;

    // auto bfield_grad = grad(_current_estimate,
    //     bfield_world_frame,
    //     magnet_m);

    // auto total_grad = gravity_grad + bfield_grad;
    // auto normalised_total_grad = total_grad / total_grad.norm();

    // auto err = 2.0f * _current_estimate.conjugate() * normalised_total_grad;
    // _estimated_bias += _bias_convergence_rate * err.im() * time_delta;

    // _current_estimate += (current_estimate_derivative - _beta * normalised_total_grad) * time_delta;
    // _current_estimate /= _current_estimate.norm();
}

Quaternion& MahonyFilter::estimate()
{
    return _current_estimate;
}
}