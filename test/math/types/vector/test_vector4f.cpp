#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VectorTest"
#define BOOST_TEST_MAIN
#include "test_util.h"
#include "vector4f.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_vector4f_empty_construction)
{
    FrameDrag::Vector4f v{};
    TEST_CHECK_FLOAT_VALUE(v[0], 0.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[1], 0.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[2], 0.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[3], 0.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_list_construction)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    TEST_CHECK_FLOAT_VALUE(v[0], 1.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[1], 2.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[2], 3.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[3], 4.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_copy_operator)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    FrameDrag::Vector4f v2{ 5.0f, 1.0f, 8.0f, 23.1f };

    FrameDrag::Vector4f& ref = (v = v2);
    v2[1] = 0.0f;
    TEST_CHECK_FLOAT_VALUE(v[0], 5.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[1], 1.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[2], 8.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[3], 23.1f, Test::EPS);

    TEST_CHECK_FLOAT_VALUE(v[0], 5.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(ref[0], 5.0f, Test::EPS);
    v[0] = 6.0f;
    TEST_CHECK_FLOAT_VALUE(v[0], 6.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(ref[0], 6.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_addition)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    FrameDrag::Vector4f v2{ 3.0f, 4.0f, 5.0f, 11.1f };
    auto v3 = v + v2;
    TEST_CHECK_FLOAT_VALUE(v3[0], v[0] + v2[0], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[1], v[1] + v2[1], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[2], v[2] + v2[2], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[3], v[3] + v2[3], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_unary_subtraction)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    auto v2 = -v;
    TEST_CHECK_FLOAT_VALUE(v2[0], -(v[0]), Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[1], -(v[1]), Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[2], -(v[2]), Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[3], -(v[3]), Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_self_addition)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    FrameDrag::Vector4f v2{ 3.0f, 4.0f, 5.0f, 6.0f };

    auto v_address = &v;

    v += v2;

    BOOST_CHECK_EQUAL(v_address, &v);

    TEST_CHECK_FLOAT_VALUE(v[0], 4.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[1], 6.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[2], 8.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[3], 10.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_self_subtraction)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    FrameDrag::Vector4f v2{ 3.0f, 4.0f, 5.0f, 6.0f };

    auto v_address = &v;

    v -= v2;

    BOOST_CHECK_EQUAL(v_address, &v);

    TEST_CHECK_FLOAT_VALUE(v[0], -2.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[1], -2.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[2], -2.0f, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v[3], -2.0f, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_subtraction)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    FrameDrag::Vector4f v2{ 3.0f, 4.0f, 5.0f, 6.0f };
    auto v3 = v - v2;
    TEST_CHECK_FLOAT_VALUE(v3[0], v[0] - v2[0], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[1], v[1] - v2[1], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[2], v[2] - v2[2], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v3[3], v[3] - v2[3], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_scalar_multiplication)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    float scalar = 56.2f;
    auto v2 = scalar * v;
    TEST_CHECK_FLOAT_VALUE(v2[0], scalar * v[0], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[1], scalar * v[1], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[2], scalar * v[2], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[3], scalar * v[3], Test::EPS);

    v2 = v * scalar;
    TEST_CHECK_FLOAT_VALUE(v2[0], scalar * v[0], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[1], scalar * v[1], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[2], scalar * v[2], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[3], scalar * v[3], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_scalar_division)
{
    FrameDrag::Vector4f v{ 1.0f, 2.0f, 3.0f, 4.0f };
    float scalar = 56.2f;
    auto v2 = v / scalar;
    TEST_CHECK_FLOAT_VALUE(v2[0], v[0] / scalar, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[1], v[1] / scalar, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[2], v[2] / scalar, Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[3], v[3] / scalar, Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_vector4f_reverse)
{
    FrameDrag::Vector4f v{ 1.0f, 6.0f, 34.1f, 23.3f };
    auto v2 = v.reverse();
    TEST_CHECK_FLOAT_VALUE(v2[0], v[3], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[1], v[2], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[2], v[1], Test::EPS);
    TEST_CHECK_FLOAT_VALUE(v2[3], v[0], Test::EPS);
}

BOOST_AUTO_TEST_CASE(test_inner_product)
{
    FrameDrag::Vector4f v{1.0f, 5.4f, 0.1f, 3.4f};
    FrameDrag::Vector4f v2{9.5f, 1.5f, 90.2f, 7.1f};
    TEST_CHECK_FLOAT_VALUE(v.innerProduct(v2), v[0]*v2[0] + v[1]*v2[1] + v[2]*v2[2] + v[3]*v2[3], Test::EPS);
}