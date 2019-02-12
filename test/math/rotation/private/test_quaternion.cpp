#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "QuaternionTest"
#define BOOST_TEST_MAIN
#include "quaternion.h"
#include "test_util.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_zero_angle_quat) {
  FrameDrag::Quaternion q{0.0f, {1.0f, 0.0f, 0.0f}};
  auto v = FrameDrag::Vector3f{1.0f, 0.0f, 0.0f};
  auto v2 = q.apply(v);
  BOOST_CHECK_EQUAL(v, v2);

  FrameDrag::Quaternion q2{0.0f, {0.0f, 1.0f, 0.0f}};
  auto vv = FrameDrag::Vector3f{1.0f, 0.0f, 0.0f};
  auto vv2 = q.apply(vv);
  BOOST_CHECK_EQUAL(vv, vv2);

  FrameDrag::Quaternion q3{0.0f, {0.0f, 0.0f, 1.0f}};
  auto vvv = FrameDrag::Vector3f{1.0f, 0.0f, 0.0f};
  auto vvv2 = q.apply(vvv);
  BOOST_CHECK_EQUAL(vvv, vvv2);
}

BOOST_AUTO_TEST_CASE(test_pi_angle_quat) {
  FrameDrag::Quaternion q{std::cos(3.14159265f / 2.0f),
                          sin(3.14159265f / 2.0f) *
                              FrameDrag::Vector3f{1.0f, 0.0f, 0.0f}};
  auto v = FrameDrag::Vector3f{0.0f, 0.0f, 1.0f};
  auto v2 = q.apply(v);
  TEST_CHECK_FLOAT_VALUE(v2[0], 0.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(v2[1], 0.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(v2[2], -v[2], 0.0001f);

  FrameDrag::Quaternion q2{std::cos(3.14159265f / 2.0f),
                           std::sin(3.14159265f / 2.0f) *
                               FrameDrag::Vector3f{0.0f, 1.0f, 0.0f}};
  auto vv = FrameDrag::Vector3f{0.0f, 1.0f, 0.0f};
  auto vv2 = q2.apply(vv);
  TEST_CHECK_FLOAT_VALUE(vv2[0], vv[0], 0.0001f);
  TEST_CHECK_FLOAT_VALUE(vv2[1], vv[1], 0.0001f);
  TEST_CHECK_FLOAT_VALUE(vv2[2], vv[2], 0.0001f);

  FrameDrag::Quaternion q3{std::cos(3.14159265f / 2.0f),
                           std::sin(3.14159265f / 2.0f) *
                               FrameDrag::Vector3f{0.0f, 0.0f, 1.0f}};
  auto vvv = FrameDrag::Vector3f{1.0f, 0.0f, 0.0f};
  auto vvv2 = q3.apply(vvv);
  TEST_CHECK_FLOAT_VALUE(vvv2[0], -vvv[0], 0.0001f);
  TEST_CHECK_FLOAT_VALUE(vvv2[1], 0.0f, 0.0001f);
  TEST_CHECK_FLOAT_VALUE(vvv2[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_inverse){
  FrameDrag::Quaternion q{3.0f, {1.5f, 6.8f, 9.1f}};
  auto v = FrameDrag::Vector3f{3.2f, 78.4f, 99.2f};
  auto v2 = q.apply(v);
  auto v3 = q.inverse().apply(v2);
  TEST_CHECK_FLOAT_VALUE(v[0], v3[0], 0.0001f);
  TEST_CHECK_FLOAT_VALUE(v[1], v3[1], 0.0001f);
  TEST_CHECK_FLOAT_VALUE(v[2], v3[2], 0.0001f);
}