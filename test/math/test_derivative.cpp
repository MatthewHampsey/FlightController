#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "DerivativeTest"
#define BOOST_TEST_MAIN
#include "derivative.h"
#include "test_util.h"
#include "vector3f.h"
#include <boost/test/included/unit_test.hpp>
#include <cmath>
#include <iostream>

BOOST_AUTO_TEST_CASE(test_vector_numerical_derivative)
{
    FrameDrag::Vector3f v1{ 1.0f, 2.0f, 3.0f };
    FrameDrag::Vector3f v2{ 3.0f, 6.0f, 9.0f };
    std::cout << "hey mofo im talking to you" << '\n';
    FrameDrag::Vector3f d = FrameDrag::numerical_derivative(v1, v2, 1.0f);

    TEST_CHECK_FLOAT_VALUE(d[0], 2.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(d[1], 4.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(d[2], 6.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_float_numerical_derivative)
{
    float f1 = 5.0f;
    float f2 = 9.0f;
    float d = FrameDrag::numerical_derivative(f1, f2, 0.5f);

    TEST_CHECK_FLOAT_VALUE(d, 8.0f, Test::EPS);
}
