#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "MatrixTest"
#define BOOST_TEST_MAIN
#include "matrix.h"
#include "test_util.h"
#include "vector3f.h"
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(test_identity) {
  FrameDrag::Matrix3f m{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  FrameDrag::Vector3f in{0.0f, 1.5f, 20.6f};
  FrameDrag::Vector3f out = m * in;

  TEST_CHECK_FLOAT_VALUE(in[0], out[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(in[1], out[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(in[2], out[2], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_matrix_multiplication) {
  FrameDrag::Matrix3f m{1.0f, 1.0f, 1.0f, 0.0f,   1.0f,
                        0.0f, 5.0f, 0.0f, 2000.0f};
  FrameDrag::Vector3f in{0.0f, 1.5f, 20.6f};
  FrameDrag::Vector3f out = m * in;

  TEST_CHECK_FLOAT_VALUE(in[0] + in[1] + in[2], out[0], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(in[1], out[1], Test::EPS);
  TEST_CHECK_FLOAT_VALUE(5.0f * in[0] + 2000.0f * in[2], out[2], Test::EPS);
}