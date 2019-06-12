#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "SensorTest"
#define BOOST_TEST_MAIN
#include "mahony_complimentary_filter.h"
#include "test_util.h"
#include <boost/test/included/unit_test.hpp>
#include <cmath>
#include <random>
#include <iostream>
#include <utility>
#include "common.h"
#include <iostream>
#include "../../../math/types/type_conversion.h"

BOOST_AUTO_TEST_CASE(mahony_filter_marg_measurements)
{
    FrameDrag::Quaternion initial_quat{1.0f, 1.0f, 1.0f, 1.0f};
    initial_quat /= initial_quat.norm();

    FrameDrag::Vector3f ref_mag{ 4.0f, 0.0f, 12.0f };
    ref_mag/ref_mag.norm();

    FrameDrag::MahonyFilter filter{
        initial_quat,
        {std::make_pair(5.0f, FrameDrag::Vector3f{0.0, 0.0, -1.0f}), 
         std::make_pair(4.0f, ref_mag)},
        3.0f, 1.0f
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
            {grav_m/grav_m.norm(),
            magn_m/magn_m.norm()},
            delta_t);
                    //std::cout << filter.estimate().apply(grav_m/grav_m.norm()) << '\n';

    }
    TEST_CHECK_FLOAT_VALUE(filter.estimate().re(), actual_ori.re(), 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[0], actual_ori.im()[0], 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[1], actual_ori.im()[1], 0.005f);
    TEST_CHECK_FLOAT_VALUE(filter.estimate().im()[2], actual_ori.im()[2], 0.005f);
}

BOOST_AUTO_TEST_CASE(mahony_filter_imu_measurements)
{
    FrameDrag::Quaternion initial_quat{1.0f, 1.0f, 1.0f, 1.0f};
    initial_quat /= initial_quat.norm();

    FrameDrag::MahonyFilter filter{
        initial_quat,
        {std::make_pair(1.0f, FrameDrag::Vector3f{0.0, 0.0, -1.0f})},
        40.0f, 0.002f
    };

    float angle = 2.1f;
    FrameDrag::Quaternion actual_ori{ std::cos(angle / 2.0f),
        std::sin(angle / 2.0f) * FrameDrag::Vector3f{ 1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f) } };
    FrameDrag::Vector3f bias{0.3f, 0.6f, 0.01f};
    for (int i = 0; i < 1000; i++) {
        FrameDrag::Vector3f actual_ang_vel = randomVector(0.000);
        float delta_t = 0.01f;
        //actual_ori += 0.5 * actual_ori * FrameDrag::Quaternion{ 0.0f, actual_ang_vel } * delta_t;
        actual_ori /= actual_ori.norm();
        FrameDrag::Vector3f gyro_m = actual_ang_vel + randomVector(0.0000f) + bias;
        FrameDrag::Vector3f grav_m = actual_ori.conjugate().apply({ 0.0f, 0.0f, -9.8f }) + randomVector(0.0000f);
        filter.update(gyro_m,
            {grav_m/grav_m.norm()},
            delta_t);

       // std::cout << filter.estimate().apply(grav_m/grav_m.norm()) << '\n';

    }
   // auto orientation = FrameDrag::quaternionToRotationMatrix(filter.estimate());
   //std::cout << orientation << '\n' << '\n';;
   auto estimated_measurement = filter.estimate().conjugate().apply(FrameDrag::Vector3f{0.0, 0.0, -1.0f});
   // std::cout << FrameDrag::quaternionToRotationMatrix(actual_ori) << '\n' << '\n';
   auto actual_measurement = actual_ori.conjugate().apply(FrameDrag::Vector3f{0.0, 0.0, -1.0f});

    TEST_CHECK_FLOAT_VALUE(estimated_measurement[0], actual_measurement[0], 0.01f);
    TEST_CHECK_FLOAT_VALUE(estimated_measurement[1], actual_measurement[1], 0.01f);
    TEST_CHECK_FLOAT_VALUE(estimated_measurement[2], actual_measurement[2], 0.01f);
}