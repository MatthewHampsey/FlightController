#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "EulerTest"
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include "euler.h"
#include "test_util.h"
#include <cmath>
#include <iostream>

BOOST_AUTO_TEST_CASE(test_yaw) {
    FrameDrag::Rotation yaw_rotation = FrameDrag::ZYXEulerToRotation(std::atan(1)*4, 0.0f, 0.0f);
    FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
    auto x_hat_rot = yaw_rotation.apply(x_hat);

    TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
    auto y_hat_rot = yaw_rotation.apply(y_hat);

    TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
    auto z_hat_rot = yaw_rotation.apply(z_hat);

    TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], 1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_pitch) {
    FrameDrag::Rotation pitch_rotation = FrameDrag::ZYXEulerToRotation(0.0f, std::atan(1)*4, 0.0f);
    FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
    auto x_hat_rot = pitch_rotation.apply(x_hat);

    TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
    auto y_hat_rot = pitch_rotation.apply(y_hat);

    TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], 1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
    auto z_hat_rot = pitch_rotation.apply(z_hat);

    TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_roll) {
    FrameDrag::Rotation roll_rotation = FrameDrag::ZYXEulerToRotation(0.0f, 0.0f, std::atan(1)*4);
    FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
    auto x_hat_rot = roll_rotation.apply(x_hat);

    TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], 1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
    auto y_hat_rot = roll_rotation.apply(y_hat);

    TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
    auto z_hat_rot = roll_rotation.apply(z_hat);

    TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_pitch_then_yaw) {
    FrameDrag::Rotation roll_rotation = FrameDrag::ZYXEulerToRotation(std::atan(1)*4, std::atan(1), 0.0f);
    FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
    auto x_hat_rot = roll_rotation.apply(x_hat);

    float forty_five_degrees = 1/std::sqrt(2);

    TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], -forty_five_degrees, 0.001f);

    FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
    auto y_hat_rot = roll_rotation.apply(y_hat);

    TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
    auto z_hat_rot = roll_rotation.apply(z_hat);

    TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], -forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], forty_five_degrees, 0.001f);
}

BOOST_AUTO_TEST_CASE(test_roll_then_yaw) {
    FrameDrag::Rotation roll_rotation = FrameDrag::ZYXEulerToRotation(std::atan(1), 0.0f, std::atan(1)*4);
    FrameDrag::Vector3f x_hat{1.0f, 0.0f, 0.0f};
    auto x_hat_rot = roll_rotation.apply(x_hat);

    const float forty_five_degrees = 1/std::sqrt(2);

    TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f y_hat{0.0f, 1.0f, 0.0f};
    auto y_hat_rot = roll_rotation.apply(y_hat);

    TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -forty_five_degrees, 0.001f);
    TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

    FrameDrag::Vector3f z_hat{0.0f, 0.0f, 1.0f};
    auto z_hat_rot = roll_rotation.apply(z_hat);

    TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
    TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
}