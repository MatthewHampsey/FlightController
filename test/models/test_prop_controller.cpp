#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "PropellorControllerTest"
#define BOOST_TEST_MAIN
#include "prop_controller.h"
#include "test_util.h"
#include <boost/test/unit_test.hpp>
#include <cmath>

BOOST_AUTO_TEST_CASE(check_default_speeds)
{

    FrameDrag::PropellerController pc(1.0f, 1.0f, 1.0f);
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], 0.0f, 0.0001f);

    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_thrust_no_torque)
{
    float k_f = 2.3f;
    FrameDrag::PropellerController pc(1.0f, k_f, 1.0f);
    pc.applyControlTargets(1.0f, { 0.0f, 0.0f, 0.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], 1.0f / (4.0f * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 1.0f / (4.0f * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 1.0f / (4.0f * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], 1.0f / (4.0f * k_f), 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 1.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_roll)
{
    float k_f = 3.5f;
    float length = 1.2f;
    FrameDrag::PropellerController pc(length, k_f, 1.0f);
    pc.applyControlTargets(0.0f, { 1.0f, 0.0f, 0.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], -1.0f / (2.0f * length * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 1.0f / (2.0f * length * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], 0.0f, 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_pitch)
{
    float k_f = 7.1f;
    float length = 0.3f;
    FrameDrag::PropellerController pc(length, k_f, 1.0f);
    pc.applyControlTargets(0.0f, { 0.0f, -1.0f, 0.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 1.0f / (2.0f * length * k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], -1.0f / (2.0f * length * k_f), 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], -1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_yaw)
{
    float k_f = 7.1f;
    float length = 0.3f;
    float k_t = 2.3f;
    FrameDrag::PropellerController pc(length, k_f, k_t);
    pc.applyControlTargets(0.0f, { 0.0f, 0.0f, 1.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], 1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], -1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], -1.0f / (4.0f * k_t), 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 1.0f, 0.0001f);

    pc.applyControlTargets(0.0f, { 0.0f, 0.0f, -4.0f });
    speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], -1.0f / k_t, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 1.0f / k_t, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], -1.0f / k_t, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], 1.0f / k_t, 0.0001f);
    thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], -4.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_yaw_with_roll)
{
    float k_f = 7.1f;
    float length = 0.3f;
    float k_t = 2.3f;
    FrameDrag::PropellerController pc(length, k_f, k_t);
    pc.applyControlTargets(0.0f, { 1.0f, 0.0f, 1.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(
        speed[0], -1.0f / (2.0f * length * k_f) + 1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], -1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(
        speed[2], 1.0f / (2.0f * length * k_f) + 1.0f / (4.0f * k_t), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], -1.0f / (4.0f * k_t), 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 0.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 1.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(check_roll_with_thrust)
{
    float k_f = 7.1f;
    float length = 0.3f;
    float k_t = 2.3f;
    FrameDrag::PropellerController pc(length, k_f, k_t);
    pc.applyControlTargets(4.0f, { 1.0f, 0.0f, 0.0f });
    FrameDrag::Vector4f speed = pc.getCurrentPropSpeed();
    TEST_CHECK_FLOAT_VALUE(speed[0], -1.0f / (2.0f * length * k_f) + 1.0f / (k_f),
        0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[1], 1.0f / (k_f), 0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[2], 1.0f / (2.0f * length * k_f) + 1.0f / (k_f),
        0.0001f);
    TEST_CHECK_FLOAT_VALUE(speed[3], 1.0f / (k_f), 0.0001f);
    float thrust = pc.getCurrentThrust();
    TEST_CHECK_FLOAT_VALUE(thrust, 4.0f, 0.0001f);

    FrameDrag::Vector3f torque = pc.getCurrentTorque();
    TEST_CHECK_FLOAT_VALUE(torque[0], 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(torque[2], 0.0f, 0.0001f);
}