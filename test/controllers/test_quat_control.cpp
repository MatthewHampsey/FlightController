#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "ControllerTest"
#define BOOST_TEST_MAIN
#include "quat_control.h"
#include "test_util.h"
#include <boost/test/included/unit_test.hpp>
#include <cmath>

FrameDrag::Quaternion ZYXEulerToQuaternion(const FrameDrag::Vector3f& v) {
  const auto y2 = v[2] / 2.0f;
  const auto p2 = v[1] / 2.0f;
  const auto r2 = v[0] / 2.0f;
  const auto cosy2 = std::cos(y2);
  const auto siny2 = std::sin(y2);
  const auto cosp2 = std::cos(p2);
  const auto sinp2 = std::sin(p2);
  const auto cosr2 = std::cos(r2);
  const auto sinr2 = std::sin(r2);
  return FrameDrag::Quaternion(cosy2 * cosp2 * cosr2 + siny2 * sinp2 * sinr2,
                               cosy2 * cosp2 * sinr2 - siny2 * sinp2 * cosr2,
                               cosy2 * sinp2 * cosr2 + siny2 * cosp2 * sinr2,
                               siny2 * cosp2 * cosr2 - cosy2 * sinp2 * sinr2);
}

BOOST_AUTO_TEST_CASE(test_control_vector) {
  auto I =
      FrameDrag::Matrix3f{2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 4.0f};
  FrameDrag::QuaternionController controller(I);
  // controller.setTimeConstant(1.0f);

  FrameDrag::Vector3f euler_angles{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f target_euler_angles{10.0f, 10.0f, 10.0f};
  auto current_att = ZYXEulerToQuaternion(euler_angles);
  auto target_att = ZYXEulerToQuaternion(target_euler_angles);
  //   FrameDrag::Vector3f control_vector = controller.getControlVector(
  //       current_att, target_att);
}
