#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VectorTest"
#define BOOST_TEST_MAIN
#include "test_util.h"
#include "vector3f.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_vector_empty_construction) {
  FrameDrag::Vector3f v{};
  TEST_CHECK_FLOAT_VALUE(v[0], 0.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[1], 0.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[2], 0.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_list_construction) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  TEST_CHECK_FLOAT_VALUE(v[0], 1.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[1], 2.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[2], 3.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_copy_operator) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f v2{5.0f, 1.0f, 8.0f};

  FrameDrag::Vector3f& ref = (v = v2);
  v2[1] = 0.0f;
  TEST_CHECK_FLOAT_VALUE(v[0], 5.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[1], 1.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[2], 8.0f, Test::EPS);

  TEST_CHECK_FLOAT_VALUE(v[0], 5.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(ref[0], 5.0f, Test::EPS);
  v[0] = 6.0f;
  TEST_CHECK_FLOAT_VALUE(v[0], 6.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(ref[0], 6.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_addition) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f v2{3.0f, 4.0f, 5.0f};
  auto v3 = v + v2;
  TEST_CHECK_FLOAT_VALUE(v3[0], v[0] + v2[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v3[1], v[1] + v2[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v3[2], v[2] + v2[2], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_unary_subtraction) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  auto v2 = -v;
  TEST_CHECK_FLOAT_VALUE(v2[0], -(v[0]), Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[1], -(v[1]), Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[2], -(v[2]), Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_self_addition) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f v2{3.0f, 4.0f, 5.0f};

  auto v_address = &v;

  v += v2;

  BOOST_CHECK_EQUAL(v_address, &v);

  TEST_CHECK_FLOAT_VALUE(v[0], 4.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[1], 6.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[2], 8.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_self_subtraction) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f v2{3.0f, 4.0f, 5.0f};

  auto v_address = &v;

  v -= v2;

  BOOST_CHECK_EQUAL(v_address, &v);

  TEST_CHECK_FLOAT_VALUE(v[0], -2.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[1], -2.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v[2], -2.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_subtraction) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  FrameDrag::Vector3f v2{3.0f, 4.0f, 5.0f};
  auto v3 = v - v2;
  TEST_CHECK_FLOAT_VALUE(v3[0], v[0] - v2[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v3[1], v[1] - v2[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v3[2], v[2] - v2[2], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_scalar_multiplication) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  float scalar = 56.2f;
  auto v2 = scalar * v;
  TEST_CHECK_FLOAT_VALUE(v2[0], scalar * v[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[1], scalar * v[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[2], scalar * v[2], Test::EPS);

  v2 = v * scalar;
  TEST_CHECK_FLOAT_VALUE(v2[0], scalar * v[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[1], scalar * v[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[2], scalar * v[2], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector_scalar_division) {
  FrameDrag::Vector3f v{1.0f, 2.0f, 3.0f};
  float scalar = 56.2f;
  auto v2 = v / scalar;
  TEST_CHECK_FLOAT_VALUE(v2[0], v[0] / scalar, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[1], v[1] / scalar, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[2], v[2] / scalar, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_reverse) {
  FrameDrag::Vector3f v{1.0f, 6.0f, 34.1f};
  auto v2 = v.reverse();
  float scalar = 3.0f;
  TEST_CHECK_FLOAT_VALUE(v2[0], v[2], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[1], v[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(v2[2], v[0], Test::EPS);
}