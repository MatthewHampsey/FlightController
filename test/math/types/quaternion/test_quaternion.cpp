#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "QuaternionTest"
#define BOOST_TEST_MAIN
#include "quaternion.h"
#include "test_util.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/included/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_zero_angle_quat)
{
    FrameDrag::Quaternion q{ 0.0f, { 1.0f, 0.0f, 0.0f } };
    auto v = FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f };
    auto v2 = q.apply(v);
    BOOST_CHECK_EQUAL(v, v2);

    FrameDrag::Quaternion q2{ 0.0f, { 0.0f, 1.0f, 0.0f } };
    auto vv = FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f };
    auto vv2 = q.apply(vv);
    BOOST_CHECK_EQUAL(vv, vv2);

    FrameDrag::Quaternion q3{ 0.0f, { 0.0f, 0.0f, 1.0f } };
    auto vvv = FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f };
    auto vvv2 = q.apply(vvv);
    BOOST_CHECK_EQUAL(vvv, vvv2);
}

BOOST_AUTO_TEST_CASE(test_pi_angle_quat)
{
    FrameDrag::Quaternion q{ std::cos(3.14159265f / 2.0f),
        sin(3.14159265f / 2.0f) * FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f } };
    auto v = FrameDrag::Vector3f{ 0.0f, 0.0f, 1.0f };
    auto v2 = q.apply(v);
    TEST_CHECK_FLOAT_VALUE(v2[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[2], -v[2], 0.0001f);

    FrameDrag::Quaternion q2{ std::cos(3.14159265f / 2.0f),
        std::sin(3.14159265f / 2.0f) * FrameDrag::Vector3f{ 0.0f, 1.0f, 0.0f } };
    auto vv = FrameDrag::Vector3f{ 0.0f, 1.0f, 0.0f };
    auto vv2 = q2.apply(vv);
    TEST_CHECK_FLOAT_VALUE(vv2[0], vv[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vv2[1], vv[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vv2[2], vv[2], 0.0001f);

    FrameDrag::Quaternion q3{ std::cos(3.14159265f / 2.0f),
        std::sin(3.14159265f / 2.0f) * FrameDrag::Vector3f{ 0.0f, 0.0f, 1.0f } };
    auto vvv = FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f };
    auto vvv2 = q3.apply(vvv);
    TEST_CHECK_FLOAT_VALUE(vvv2[0], -vvv[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_inverse)
{
    FrameDrag::Quaternion q{ 3.0f, { 1.5f, 6.8f, 9.1f } };
    TEST_CHECK_FLOAT_VALUE((q * q.inverse()).re(), 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.inverse()).im()[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.inverse()).im()[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.inverse()).im()[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_conjugate)
{
    FrameDrag::Quaternion q{ 3.0f, { 1.5f, 6.8f, 9.1f } };
    TEST_CHECK_FLOAT_VALUE((q * q.conjugate()).re(), 9.0f + 1.5f * 1.5f + 6.8f * 6.8f + 9.1f * 9.1f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.conjugate()).im()[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.conjugate()).im()[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE((q * q.conjugate()).im()[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_accessors)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    TEST_CHECK_FLOAT_VALUE(q.re(), 3.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q.im()[0], 1.5f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q.im()[1], 6.8f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q.im()[2], 9.1f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_scalar_multiplication)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    float scalar = 4.2f;
    FrameDrag::Quaternion q2 = scalar * q;
    TEST_CHECK_FLOAT_VALUE(q2.re(), scalar * q.re(), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[0], scalar * q.im()[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[1], scalar * q.im()[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[2], scalar * q.im()[2], 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_quaternion_multiplication)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    FrameDrag::Quaternion q2{ 5.6f, 1.2f, 9.11f, 78.9f };
    FrameDrag::Quaternion q3 = q * q2;

    TEST_CHECK_FLOAT_VALUE(q3.re(), q.re() * q2.re() - q.im().innerProduct(q2.im()), 0.0001f);

    auto scaled_q = q2.re() * q.im();
    auto scaled_q2 = q.re() * q2.im();
    auto cross_prod = q.im().cross(q2.im());
    auto result = scaled_q + scaled_q2 + cross_prod;

    TEST_CHECK_FLOAT_VALUE(q3.im()[0], result[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[1], result[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[2], result[2], 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_scalar_division)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    float scalar = 4.2f;
    FrameDrag::Quaternion q2 = q / scalar;
    TEST_CHECK_FLOAT_VALUE(q2.re(), q.re() / scalar, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[0], q.im()[0] / scalar, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[1], q.im()[1] / scalar, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q2.im()[2], q.im()[2] / scalar, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_quaternion_addition)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    FrameDrag::Quaternion q2{ 5.1f, 4.1f, 6.0f, 92.4f };
    auto q3 = q + q2;
    TEST_CHECK_FLOAT_VALUE(q3.re(), q.re() + q2.re(), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[0], q.im()[0] + q2.im()[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[1], q.im()[1] + q2.im()[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[2], q.im()[2] + q2.im()[2], 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_quaternion_subtraction)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 9.1f };
    FrameDrag::Quaternion q2{ 5.1f, 4.1f, 6.0f, 92.4f };
    auto q3 = q - q2;
    TEST_CHECK_FLOAT_VALUE(q3.re(), q.re() - q2.re(), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[0], q.im()[0] - q2.im()[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[1], q.im()[1] - q2.im()[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q3.im()[2], q.im()[2] - q2.im()[2], 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_apply)
{
    FrameDrag::Quaternion q{ 3.0f, 1.5f, 6.8f, 7.1f };
    FrameDrag::Vector3f vec{ 5.67f, 14.1, 2.9f };
    auto result = q.apply(vec);
    auto conj_result = (q * FrameDrag::Quaternion(0.0f, vec) * q.conjugate()).im();
    TEST_CHECK_FLOAT_VALUE(result[0], conj_result[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(result[1], conj_result[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(result[2], conj_result[2], 0.0001f);
}