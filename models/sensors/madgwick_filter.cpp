#include "madgwick_filter.h"
#include "matrix.h"
#include "vector4f.h"

namespace FrameDrag {

MadgwickFilter::MadgwickFilter()
    : _current_estimate{ 0.0f, 0.0f, 0.0f, 0.0f }
    , _beta{ 0.0f }
{
}

MadgwickFilter::MadgwickFilter(const Quaternion& q, float beta)
    : _current_estimate{ q }
    , _beta{ beta }
{
}

MadgwickFilter::MadgwickFilter(Quaternion&& q, float beta)
    : _current_estimate{ std::move(q) }
    , _beta{ beta }
{
}

void MadgwickFilter::update(const Vector3f& gyro_m,
    const Vector3f& accel_m,
    const Vector3f& magnet_m,
    float time_delta)
{
    Quaternion current_estimate_derivative = 0.5 * _current_estimate * Quaternion(0.0f, gyro_m);

    auto gravity_grad = grad(_current_estimate,
        { 0.0f, 0.0f, -9.8f },
        accel_m);
    auto bfield_grad = grad(_current_estimate,
        { 1, 0, 1 },
        magnet_m);
    auto total_grad = gravity_grad + bfield_grad;
    auto normalised_total_grad = _beta * total_grad / total_grad.norm();

    _current_estimate += (current_estimate_derivative - normalised_total_grad) * time_delta;
}

void MadgwickFilter::update(const Vector3f& gyro_m,
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
}

Quaternion grad(const Quaternion& sensor_frame_to_world,
    const Vector3f& v,
    const Vector3f& measurement)
{
    const float q0 = sensor_frame_to_world.re();
    const float q1 = sensor_frame_to_world.im()[0];
    const float q2 = sensor_frame_to_world.im()[1];
    const float q3 = sensor_frame_to_world.im()[2];

    // Matrix4f jacobian{2*(q3*v[1] - q2*v[2]), 2*(q2*v[1] + q3*v[2]), 2*(q1*v[1] - q0*v[2]) - 4*q2*v[0], 2*(q1*v[2] + q0*v[1]) -4*q3*v[0],
    //                   2*(q1*v[2] - q3*v[0]), 2*(q2*v[0] + q0*v[2]) - 4*q1*v[1], 2*(q3*v[2] + q1*v[0]), 2*(q2*v[2] - q0*v[0]) - 4*q3*v[1],
    //                   2*(q2*v[0] - q1*v[1]), 2*(q3*v[0] - q0*v[1]) - 4*q1*v[2], 2*(q3*v[1] + q0*v[0]) - 4*q2*v[2], 2*(q1*v[0] + q2*v[1]),
    //                   0.0f, 0.0f, 0.0f, 0.0f};
    Matrix4f jacobian_transpose{
        2 * (q3 * v[1] - q2 * v[2]), 2 * (q1 * v[2] - q3 * v[0]), 2 * (q2 * v[0] - q1 * v[1]), 0.0f,
        2 * (q2 * v[1] + q3 * v[2]), 2 * (q2 * v[0] + q0 * v[2]) - 4 * q1 * v[1], 2 * (q3 * v[0] - q0 * v[1]) - 4 * q1 * v[2], 0.0f,
        2 * (q1 * v[1] - q0 * v[2]) - 4 * q2 * v[0], 2 * (q3 * v[2] + q1 * v[0]), 2 * (q3 * v[1] + q0 * v[0]) - 4 * q2 * v[2], 0.0f,
        2 * (q1 * v[2] + q0 * v[1]) - 4 * q3 * v[0], 2 * (q2 * v[2] - q0 * v[0]) - 4 * q3 * v[1], 2 * (q1 * v[0] + q2 * v[1]), 0.0f
    };
    auto temp = sensor_frame_to_world.conjugate().apply(v) - measurement;
    Vector4f temp_q{ temp[0], temp[1], temp[2], 0.0f };
    Vector4f result = jacobian_transpose * temp_q;

    return Quaternion{ result[0], result[1], result[2], result[3] };
}

Quaternion& MadgwickFilter::estimate()
{
    return _current_estimate;
}

void MadgwickFilter::setBeta(float beta)
{
    _beta = beta;
}
}