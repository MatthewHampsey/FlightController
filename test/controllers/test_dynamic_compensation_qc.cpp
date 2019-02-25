#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "ControllerTest"
#define BOOST_TEST_MAIN
#include "dynamic_compensation_qc.h"
#include "test_util.h"
#include <boost/test/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(set_control_parameters) {

  FrameDrag::PDDynamic controller(FrameDrag::Matrix3f{
      1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f});
  controller.setParameters(0.25f, 7.0f);
  auto K_p = controller.K_p();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        TEST_CHECK_FLOAT_VALUE(K_p(i, j), 49.0f, 0.0001f);
      } else {
        TEST_CHECK_FLOAT_VALUE(K_p(i, j), 0.0f, 0.0001f);
      }
    }
  }

  auto K_d = controller.K_d();
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        TEST_CHECK_FLOAT_VALUE(K_d(i, j), 3.5f, 0.0001f);
      } else {
        TEST_CHECK_FLOAT_VALUE(K_d(i, j), 0.0f, 0.0001f);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_control_vector) {
  auto I =
      FrameDrag::Matrix3f{2.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 4.0f};
  FrameDrag::PDDynamic controller(I);
  controller.setParameters(0.25f, 7.0f);

  FrameDrag::Vector3f euler_angles{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f euler_derivatives{0.3f, 0.2f, 0.1f};
  FrameDrag::Vector3f target_euler_angles{10.0f, 10.0f, 10.0f};
  FrameDrag::Vector3f target_euler_derivatives{2.0f, 2.0f, 2.0f};

  FrameDrag::Vector3f control_vector = controller.getControlVector(
      euler_angles, euler_derivatives, target_euler_angles,
      target_euler_derivatives);

  TEST_CHECK_FLOAT_VALUE(
      control_vector[0],
      I(0, 0) * (49.0f * (target_euler_angles[0] - euler_angles[0]) +
                 3.5f * (target_euler_derivatives[0] - euler_derivatives[0])),
      0.0001f);
  TEST_CHECK_FLOAT_VALUE(
      control_vector[1],
      I(1, 1) * (49.0f * (target_euler_angles[1] - euler_angles[1]) +
                 3.5f * (target_euler_derivatives[1] - euler_derivatives[1])),
      0.0001f);
  TEST_CHECK_FLOAT_VALUE(
      control_vector[2],
      I(2, 2) * (49.0f * (target_euler_angles[2] - euler_angles[2]) +
                 3.5f * (target_euler_derivatives[2] - euler_derivatives[2]) -
                 euler_derivatives[0] * euler_derivatives[1]),
      0.0001f);
}