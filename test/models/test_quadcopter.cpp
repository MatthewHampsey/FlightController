#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "QuadcopterTest"
#define BOOST_TEST_MAIN
#include "test_util.h"
#include "quadcopter.h"
#include <boost/test/unit_test.hpp>
#include <cmath>


BOOST_AUTO_TEST_CASE(quadcopter_initialisation) {

    FrameDrag::Quadcopter q{{1.0f, 2.0f, 3.0f},
                             {4.0f, 5.0f, 6.0f},
                             {7.0f, 8.0f, 9.0f},
                             {10.0f, 11.0f, 12.0f},
                             {13.0f, 14.0f, 15.0f,
                              16.0f, 17.0f, 18.0f,
                              19.0f, 20.0f, 21.0f}};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 2.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 3.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[0], 4.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[1], 5.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[2], 6.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[0], 7.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[1], 8.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[2], 9.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[0], 10.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[1], 11.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[2], 12.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(0, 0), 13.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(0, 1), 14.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(0, 2), 15.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(1, 0), 16.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(1, 1), 17.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(1, 2), 18.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(2, 0), 19.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(2, 1), 20.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._I(2, 2), 21.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_no_external_forces_quadcopter) {

    FrameDrag::Quadcopter q;
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f drag{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{0.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 0.19f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_constant_thrust_no_rotation_quadcopter) {

    FrameDrag::Quadcopter q;
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 9.81f};
    FrameDrag::Vector3f drag{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{0.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_constant_thrust_90_rotation_quadcopter) {

    FrameDrag::Quadcopter q;
    q._euler_angles = {0.0f, 3.14159f/2.0f, 0.0f};
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};


    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 9.81f};
    FrameDrag::Vector3f drag{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{0.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 9.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 9.81f, 0.0001f);
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 3.0f*9.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 3.0f*9.81f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_constant_thrust_90_rotation_quadcopter_with_drag) {

    FrameDrag::Quadcopter q;
    q._euler_angles = {0.0f, 3.14159f/2.0f, 0.0f};
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 9.81f};
    FrameDrag::Vector3f drag{-9.81f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{0.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 9.81f, 0.0001f);
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 3.0f*9.81f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_no_thrust_with_torque) {

    FrameDrag::Quadcopter q;
    q._euler_angles = {0.0f, 0.0f, 0.0f};
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f drag{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{1.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 9.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[0], 1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[2], 0.0f, 0.0001f);
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 3.0f*9.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[0], 3.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[2], 0.0f, 0.0001f);
}

BOOST_AUTO_TEST_CASE(test_thrust_with_torque) {

    FrameDrag::Quadcopter q;
    q._euler_angles = {0.0f, 0.0f, 0.0f};
    q._position = FrameDrag::Vector3f{0.0f, 0.0f, 10.0f};

    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f, 0.0001f);


    FrameDrag::Vector3f thrust{0.0f, 0.0f, 1.0f};
    FrameDrag::Vector3f drag{0.0f, 0.0f, 0.0f};
    FrameDrag::Vector3f torque{3.14159265f/2.0f, 0.0f, 0.0f};
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 8.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[2], -8.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[0], 3.14159265f/2.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[2], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[0], 3.14159265f/2.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[2], 0.0f, 0.0001f);
    q.step(1.0f, thrust, drag, torque);
    TEST_CHECK_FLOAT_VALUE(q._position[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[1], -1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._position[2], 10.0f - 9.81f -2*8.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[1], -1.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._velocity[2], -9.81f -8.81f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[0], 3.0f*3.14159265f/2.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_angles[2], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[0], 3.14159265f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(q._euler_derivatives[2], 0.0f, 0.0001f);
}