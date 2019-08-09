#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "ControllerTest"
#define BOOST_TEST_MAIN
#include "quat_control.h"
#include "test_util.h"
#include "type_conversion.h"
#include <boost/test/included/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_control_vector) {
  auto I =
      FrameDrag::Matrix3f{2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 4.0f};
  FrameDrag::QuaternionController controller(I);
  // controller.setTimeConstant(1.0f);

  FrameDrag::Vector3f euler_angles{1.0f, 0.0f, 0.0f};
  FrameDrag::Vector3f target_euler_angles{2.0f, 0.0f, 0.0f};
  auto current_att = ZYXEulerToQuaternion(euler_angles);
  FrameDrag::Vector3f ang_vel{0.0f, 0.0f, 0.0f};
  auto target_att = ZYXEulerToQuaternion(target_euler_angles);
  FrameDrag::Vector3f target_ang_vel{0.0f, 0.0f, 0.0f};
  FrameDrag::Vector3f control_vector = controller.getControlVector(
         current_att, ang_vel,  target_att, target_ang_vel);
  BOOST_CHECK(control_vector[0] > 0.0f);
  TEST_CHECK_FLOAT_VALUE(control_vector[1], 0.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(control_vector[2], 0.0f, 0.0001f);
}
