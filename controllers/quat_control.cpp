#include "quat_control.h"
#include "angular_velocity_conversion.h"

namespace FrameDrag {
QuaternionController::QuaternionController(const Matrix3f& moment_of_inertia)
    : _I{moment_of_inertia} {}

Vector3f QuaternionController::getControlVector(
    const Quaternion& q_orientation,
    const Quaternion& q_orientation_derivative,
    const Quaternion& q_target,
    const Quaternion& q_target_derivative) {
  auto q_error = q_target.conjugate() * q_orientation;
  q_error = q_error / q_error.norm();

  // orientation error = 0.5*vex(R_e - R_e^T) = sin(theta_e)*n_e
  //                  = 2*cos(theta_e/2)*sin(theta_e/2)*n_e = 2*q_0*q_v
  // where theta is the angle of rotation and n_e is the axis of rotation
  auto orientation_error = 2 * q_error.re() * q_error.im();
  auto current_angular_velocity = QuaternionToBodyFrameAngularVelocity(
      q_orientation, q_orientation_derivative);
  auto target_angular_velocity =
      QuaternionToBodyFrameAngularVelocity(q_target, q_target_derivative);
  auto angular_velocity_error =
      current_angular_velocity -
      q_error.inverse().apply(target_angular_velocity);

  auto target_acc = -angular_velocity_error - orientation_error;

  return target_acc +
         current_angular_velocity.cross(_I * current_angular_velocity);
}

void QuaternionController::setParameters(float K_omega_x,
                                         float K_omega_y,
                                         float K_omega_z,
                                         float K_R_x,
                                         float K_R_y,
                                         float K_R_z) {
  _K_omega = Matrix3f{K_omega_x, 0.0f, 0.0f, 0.0f,     K_omega_y,
                      0.0f,      0.0f, 0.0f, K_omega_z};

  _K_R = Matrix3f{K_R_x, 0.0f, 0.0f, 0.0f, K_R_y, 0.0f, 0.0f, 0.0f, K_R_z};
}

void QuaternionController::setParameters(float K_omega, float K_R) {
  setParameters(K_omega, K_omega, K_omega, K_R, K_R, K_R);
}

Matrix3f QuaternionController::K_omega() {
  return _K_omega;
}

Matrix3f QuaternionController::K_R() {
  return _K_R;
}
}
