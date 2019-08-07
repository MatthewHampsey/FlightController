#include "quat_control.h"
#include "angular_velocity_conversion.h"
#include <iostream>

namespace FrameDrag {
QuaternionController::QuaternionController(const Matrix3f& moment_of_inertia)
    : _I{ moment_of_inertia }
    , _K_omega{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0, 0.0f, 1.0f }
    , _K_R{ 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0, 0.0f, 1.0f }
{
}

    Vector3f QuaternionController::getControlVector(const Quaternion& q_orientation,
        const Vector3f& angular_velocity,
        const Quaternion& q_target,
        const Vector3f& target_angular_velocity)
{
   // std::cout << "q_orient: " << q_orientation << '\n';
   // std::cout << "q_target: " << q_target << '\n';
    auto q_error = q_target.conjugate() * q_orientation;
    q_error = q_error / q_error.norm();

    // orientation error = 0.5*vex(R_e - R_e^T) = sin(theta_e)*n_e
    //                  = 2*cos(theta_e/2)*sin(theta_e/2)*n_e = 2*q_0*q_v
    // where theta is the angle of rotation and n_e is the axis of rotation
    auto orientation_error = 2 * q_error.re() * q_error.im();
    auto angular_velocity_error = angular_velocity - q_error.inverse().apply(target_angular_velocity);
   // std::cout << "ang vel error: " << angular_velocity_error << '\n';
   // std::cout << "orientation error: " << orientation_error << '\n';
    auto target_acc = -_K_omega * angular_velocity_error - _K_R * orientation_error;

    return target_acc + angular_velocity.cross(_I * angular_velocity);
}

void QuaternionController::setParameters(float K_omega_x,
    float K_omega_y,
    float K_omega_z,
    float K_R_x,
    float K_R_y,
    float K_R_z)
{
    _K_omega = Matrix3f{ K_omega_x, 0.0f, 0.0f, 0.0f, K_omega_y,
        0.0f, 0.0f, 0.0f, K_omega_z };

    _K_R = Matrix3f{ K_R_x, 0.0f, 0.0f, 0.0f, K_R_y, 0.0f, 0.0f, 0.0f, K_R_z };
}

void QuaternionController::setParameters(float K_omega, float K_R)
{
    setParameters(K_omega, K_omega, K_omega, K_R, K_R, K_R);
}

Matrix3f QuaternionController::K_omega()
{
    return _K_omega;
}

Matrix3f QuaternionController::K_R()
{
    return _K_R;
}
}
