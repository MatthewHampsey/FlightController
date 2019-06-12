#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "AngularVelocityConversionTest"
#define BOOST_TEST_MAIN
#include "angular_velocity_conversion.h"
#include "derivative.h"
#include "quaternion.h"
#include "test_util.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/included/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(test_velocity_from_quaternion)
{
    FrameDrag::Vector3f axis{ 1 / std::sqrt(3.0f), 1 / std::sqrt(3.0f), 1 / std::sqrt(3.0f) };
    float angle = 1.2f;
    float angle2 = 1.4f;
    FrameDrag::Quaternion q(cos(angle / 2.0f), sin(angle / 2.0f) * axis);
    FrameDrag::Quaternion q2(cos(angle2 / 2.0f), sin(angle2 / 2.0f) * axis);

    auto derivative = QuaternionToBodyFrameAngularVelocity(q2, FrameDrag::numerical_derivative(q, q2, 1.0f));

    auto est_derivative = FrameDrag::numerical_derivative(angle, angle2, 1.0f) * axis;
    TEST_CHECK_FLOAT_VALUE(derivative[0], est_derivative[0], 0.01f);
    TEST_CHECK_FLOAT_VALUE(derivative[1], est_derivative[1], 0.01f);
    TEST_CHECK_FLOAT_VALUE(derivative[2], est_derivative[2], 0.01f);
}

BOOST_AUTO_TEST_CASE(test_quaternion_derivative_from_velocity)
{
    FrameDrag::Vector3f axis{ 1 / std::sqrt(3.0f), 1 / std::sqrt(3.0f), 1 / std::sqrt(3.0f) };
    float angle = 2.4f;
    float angle2 = 2.1f;
    FrameDrag::Quaternion q(cos(angle / 2.0f), sin(angle / 2.0f) * axis);
    FrameDrag::Quaternion q2(cos(angle2 / 2.0f), sin(angle2 / 2.0f) * axis);

    auto est_deriv = FrameDrag::numerical_derivative(q, q2, 1.0f);

    auto ang_velocity = FrameDrag::numerical_derivative(angle, angle2, 1.0f) * axis;

    auto q_derivative = BodyFrameAngularVelocityToQuaternionDerivative(q2,
        ang_velocity);
    TEST_CHECK_FLOAT_VALUE(est_deriv.re(), q_derivative.re(), 0.01f);
    TEST_CHECK_FLOAT_VALUE(est_deriv.im()[0], q_derivative.im()[0], 0.01f);
    TEST_CHECK_FLOAT_VALUE(est_deriv.im()[1], q_derivative.im()[1], 0.01f);
    TEST_CHECK_FLOAT_VALUE(est_deriv.im()[2], q_derivative.im()[2], 0.01f);
}