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

BOOST_AUTO_TEST_CASE(test_matrix_copy_operator) {
  FrameDrag::Matrix3f m{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};
  FrameDrag::Matrix3f m2{9.0f, 8.0f, 7.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.0f, 1.0f};

  FrameDrag::Matrix3f& ref = (m = m2);
  m2(1, 1) = 0.0f;
  TEST_CHECK_FLOAT_VALUE(m(0, 0), 9.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(0, 1), 8.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(0, 2), 7.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(1, 0), 6.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(1, 1), 5.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(1, 2), 4.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(2, 0), 3.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(2, 1), 2.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(m(2, 2), 1.0f, Test::EPS);

  TEST_CHECK_FLOAT_VALUE(m(1, 2), 4.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(ref(1, 2), 4.0f, Test::EPS);
  m(1, 2) = 6.0f;
  TEST_CHECK_FLOAT_VALUE(m(1, 2), 6.0f, Test::EPS);
  TEST_CHECK_FLOAT_VALUE(ref(1, 2), 6.0f, Test::EPS);
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

BOOST_AUTO_TEST_CASE(test_matrix_inverse) {
  FrameDrag::Matrix3f m{1.0f, 1.0f, 1.0f, 0.0f,   1.0f,
                        0.0f, 5.0f, 0.0f, 2000.0f};
  
  FrameDrag::Matrix3f m2 = m.inverse();

  FrameDrag::Matrix3f I = m*m2;

  for(int i = 0; i < 3; i ++){
    for(int j = 0; j < 3; j++){
      if(i == j){
        TEST_CHECK_FLOAT_VALUE(I(i, j), 1.0f, Test::EPS);
      }
      else{
        TEST_CHECK_FLOAT_VALUE(I(i, j), 0.0f, Test::EPS);
      }
    }
  }
}