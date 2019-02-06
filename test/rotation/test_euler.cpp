#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "EulerTest"
#define BOOST_TEST_MAIN
#include "euler.h"
#include "test_util.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <iostream>

const float forty_five_degrees = 1 / std::sqrt(2);

BOOST_AUTO_TEST_CASE(test_yaw) {
  FrameDrag::Rotation yaw_rotation =
      FrameDrag::ZYXEulerToRotation(std::atan(1) * 4, 0.0f, 0.0f);
  FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
  auto x_hat_rot = yaw_rotation.apply(x_hat);

  TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
  auto y_hat_rot = yaw_rotation.apply(y_hat);

  TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
  auto z_hat_rot = yaw_rotation.apply(z_hat);

  TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], 1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_pitch) {
  FrameDrag::Rotation pitch_rotation =
      FrameDrag::ZYXEulerToRotation(0.0f, std::atan(1) * 4, 0.0f);
  FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
  auto x_hat_rot = pitch_rotation.apply(x_hat);

  TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
  auto y_hat_rot = pitch_rotation.apply(y_hat);

  TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], 1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
  auto z_hat_rot = pitch_rotation.apply(z_hat);

  TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_roll) {
  FrameDrag::Rotation roll_rotation =
      FrameDrag::ZYXEulerToRotation(0.0f, 0.0f, std::atan(1) * 4);
  FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
  auto x_hat_rot = roll_rotation.apply(x_hat);

  TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], 1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
  auto y_hat_rot = roll_rotation.apply(y_hat);

  TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
  auto z_hat_rot = roll_rotation.apply(z_hat);

  TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_pitch_then_yaw) {
  FrameDrag::Rotation roll_rotation =
      FrameDrag::ZYXEulerToRotation(std::atan(1) * 4, std::atan(1), 0.0f);
  FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
  auto x_hat_rot = roll_rotation.apply(x_hat);

  TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], -forty_five_degrees, 0.001f);

  FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
  auto y_hat_rot = roll_rotation.apply(y_hat);

  TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
  auto z_hat_rot = roll_rotation.apply(z_hat);

  TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], -forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], forty_five_degrees, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_roll_then_yaw) {
  FrameDrag::Rotation roll_rotation =
      FrameDrag::ZYXEulerToRotation(std::atan(1), 0.0f, std::atan(1) * 4);
  FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
  auto x_hat_rot = roll_rotation.apply(x_hat);

  TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
  auto y_hat_rot = roll_rotation.apply(y_hat);

  TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -forty_five_degrees, 0.001f);
  TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

  FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
  auto z_hat_rot = roll_rotation.apply(z_hat);

  TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_x_axis) {
  FrameDrag::Vector3f e_ang{0.0f, 0.0f, 0.0f};
  FrameDrag::Vector3f e_deriv{0.0f, 0.0f, 1.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], 1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], 0.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_y_axis) {
  FrameDrag::Vector3f e_ang{0.0f, 0.0f, 0.0f};
  FrameDrag::Vector3f e_deriv{0.0f, 1.0f, 0.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], 1.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], 0.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_z_axis) {
  FrameDrag::Vector3f e_ang{0.0f, 0.0f, 0.0f};
  FrameDrag::Vector3f e_deriv{1.0f, 0.0f, 0.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], 0.0f, 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], 1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_x_axis) {
  // rotate by pi/2 around y-axis, then pi/2 around z-axis
  FrameDrag::Vector3f e_ang{3.14159265f / 2.0f, 3.14159265f / 2.0f, 0.0f};
  FrameDrag::Vector3f e_deriv{0.0f, 0.0f, 1.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  FrameDrag::Vector3f angular_velocity2 =
      e_deriv[0] * FrameDrag::Vector3f{0.0f, 0.0f, 1.0f} +
      e_deriv[1] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], 0.0f, 0.0f)
              .apply(FrameDrag::Vector3f{0.0f, 1.0f, 0.0f}) +
      e_deriv[2] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], e_ang[1], 0.0f)
              .apply(FrameDrag::Vector3f{1.0f, 0.0f, 0.0f});
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], angular_velocity2[0], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], angular_velocity2[1], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], angular_velocity2[2], 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_x_axis_2) {
  // rotate by pi/2 around y-axis, then pi/2 around z-axis
  FrameDrag::Vector3f e_ang{3.14159265f / 2.0f, 3.14159265f, 0.0f};
  FrameDrag::Vector3f e_deriv{0.0f, 0.0f, 1.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  FrameDrag::Vector3f angular_velocity2 =
      e_deriv[0] * FrameDrag::Vector3f{0.0f, 0.0f, 1.0f} +
      e_deriv[1] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], 0.0f, 0.0f)
              .apply(FrameDrag::Vector3f{0.0f, 1.0f, 0.0f}) +
      e_deriv[2] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], e_ang[1], 0.0f)
              .apply(FrameDrag::Vector3f{1.0f, 0.0f, 0.0f});
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], angular_velocity2[0], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], angular_velocity2[1], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], angular_velocity2[2], 0.001f);
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_y_axis) {
  FrameDrag::Vector3f e_ang{3.14159265f / 2.0f, 0.0f, 3.14159265f / 4.0f};
  FrameDrag::Vector3f e_deriv{0.0f, 1.0f, 0.0f};
  auto angular_velocity = ZYXEulerToAngularVelocity(e_ang, e_deriv);
  // ω    =  dyaw/dt*z +
  //         dpitch/dt*(R_z)y +
  //         droll/dt*(R_z)(R_y)x
  FrameDrag::Vector3f angular_velocity2 =
      e_deriv[0] * FrameDrag::Vector3f{0.0f, 0.0f, 1.0f} +
      e_deriv[1] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], 0.0f, 0.0f)
              .apply(FrameDrag::Vector3f{0.0f, 1.0f, 0.0f}) +
      e_deriv[2] *
          FrameDrag::ZYXEulerToRotation(e_ang[0], e_ang[1], 0.0f)
              .apply(FrameDrag::Vector3f{1.0f, 0.0f, 0.0f});
  TEST_CHECK_FLOAT_VALUE(angular_velocity[0], angular_velocity2[0], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[1], angular_velocity2[1], 0.001f);
  TEST_CHECK_FLOAT_VALUE(angular_velocity[2], angular_velocity2[2], 0.001f);
}