#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "SensorTest"
#define BOOST_TEST_MAIN
#include "madgwick_marg_filter.h"
#include "madgwick_imu_filter.h"
#include "test_util.h"
#include <boost/test/included/unit_test.hpp>
#include <cmath>
#include <random>
#include <iostream>
#include "common.h"

BOOST_AUTO_TEST_CASE(test_grad)
{
    FrameDrag::Quaternion guess{ 1.0f, 2.0f, 3.0f, 4.0f };

    FrameDrag::Vector3f known{ 0.0f, 0.0f, -9.8f };
    FrameDrag::Vector3f known2{ 2.0f, 0.0f, 4.0f };
    FrameDrag::Vector3f axis_rotation = { 1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f) };
    float theta = 2.1f;
    FrameDrag::Quaternion known_orientation_world_frame{ 
        std::cos(theta / 2.0f),
        std::sin(theta / 2.0f) * axis_rotation };
    FrameDrag::Vector3f measurement = known_orientation_world_frame.conjugate().apply(known);
    FrameDrag::Vector3f measurement2 = known_orientation_world_frame.conjugate().apply(known2);

    auto next_guess = guess;
    float factor = 1.0f;
    int next_stage = 10;
    for (int i = 0; i < 1000; i++) {
        if (i % next_stage == 0) {
            next_stage *= 10;
            factor *= 0.1f;
        }
        auto grad = FrameDrag::grad(next_guess,
            known,
            measurement);
        grad = grad + FrameDrag::grad(next_guess,
                          known2,
                          measurement2);
        next_guess = next_guess - factor * grad / grad.norm();
    }
    TEST_CHECK_FLOAT_VALUE(next_guess.re(), known_orientation_world_frame.re(), 0.01f);
    TEST_CHECK_FLOAT_VALUE(next_guess.im()[0], known_orientation_world_frame.im()[0], 0.01f);
    TEST_CHECK_FLOAT_VALUE(next_guess.im()[1], known_orientation_world_frame.im()[1], 0.01f);
    TEST_CHECK_FLOAT_VALUE(next_guess.im()[2], known_orientation_world_frame.im()[2], 0.01f);
}

BOOST_AUTO_TEST_CASE(filter_marg_measurements)
{
    FrameDrag::Quaternion initial_quat{1.0f, 1.0f, 1.0f, 1.0f};
    initial_quat /= initial_quat.norm();

    FrameDrag::MadgwickMARGFilter filter{
        initial_quat,
        1.2f,
        0.2f
    };

    float angle = 2.1f;
    FrameDrag::Quaternion actual_ori{ std::cos(angle / 2.0f),
        std::sin(angle / 2.0f) * FrameDrag::Vector3f{ 1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f) } };
    FrameDrag::Vector3f bias{0.3f, 0.6f, 0.01f};
    for (int i = 0; i < 1000; i++) {
        FrameDrag::Vector3f actual_ang_vel = randomVector(0.1);
        float delta_t = 0.01f;
        actual_ori += 0.5 * actual_ori * FrameDrag::Quaternion{ 0.0f, actual_ang_vel } * delta_t;
        FrameDrag::Vector3f gyro_m = actual_ang_vel + randomVector(0.01f) + bias;
        FrameDrag::Vector3f grav_m = actual_ori.conjugate().apply({ 0.0f, 0.0f, -9.8f }) + randomVector(0.02f);
        FrameDrag::Vector3f magn_m = actual_ori.conjugate().apply({ 4.0f, 0.0f, 12.0f }) + randomVector(0.03f);
        filter.update(gyro_m,
            grav_m,
            magn_m,
            delta_t);
    }
    TEST_CHECK_FLOAT_VALUE(filter.estimate().re(), actual_ori.re(), 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[0], actual_ori.im()[0], 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[1], actual_ori.im()[1], 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[2], actual_ori.im()[2], 0.005f);
}

BOOST_AUTO_TEST_CASE(filter_imu_measurements)
{

    FrameDrag::MadgwickIMUFilter filter{
        FrameDrag::Quaternion{ 1.0f, 1.0f, 1.0f, 1.0f },
        1.2f
    };

    float angle = 2.1f;
    FrameDrag::Quaternion actual_ori{ std::cos(angle / 2.0f),
        std::sin(angle / 2.0f) * FrameDrag::Vector3f{ 1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f) } };
    for (int i = 0; i < 1000; i++) {
        FrameDrag::Vector3f actual_ang_vel = randomVector(0.1);
        float delta_t = 0.01f;
        actual_ori += 0.5 * actual_ori * FrameDrag::Quaternion{ 0.0f, actual_ang_vel } * delta_t;
        FrameDrag::Vector3f gyro_m = actual_ang_vel + randomVector(0.001f);
        FrameDrag::Vector3f grav_m = actual_ori.conjugate().apply({ 0.0f, 0.0f, -9.8f }) + randomVector(0.002f);
        filter.update(gyro_m,
            grav_m,
            delta_t);
    }
    TEST_CHECK_FLOAT_VALUE(filter.estimate().re(), actual_ori.re(), 0.3f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[0], actual_ori.im()[0], 0.3f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[1], actual_ori.im()[1], 0.3f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[2], actual_ori.im()[2], 0.3f);
}