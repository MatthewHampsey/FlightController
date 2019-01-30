#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "QuaternionTest"
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include "quaternion.h"
#include "test_util.h"
#include <cmath>

BOOST_AUTO_TEST_CASE(test_zero_angle_quat) {
    Penguin::Quaternion q{0.0f, {1.0f, 0.0f, 0.0f}};
    auto v = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto v2 = q.apply(v);
    BOOST_CHECK_EQUAL(v, v2);
    
    Penguin::Quaternion q2{0.0f, {0.0f, 1.0f, 0.0f}};
    auto vv = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto vv2 = q.apply(vv);
    BOOST_CHECK_EQUAL(vv, vv2);

    Penguin::Quaternion q3{0.0f, {0.0f, 0.0f, 1.0f}};
    auto vvv = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto vvv2 = q.apply(vvv);
    BOOST_CHECK_EQUAL(vvv, vvv2);
}

BOOST_AUTO_TEST_CASE(test_pi_angle_quat) {
    Penguin::Quaternion q{std::cos(3.14159265f/2.0f), 
            sin(3.14159265f/2.0f)*Penguin::Vector3f{1.0f, 0.0f, 0.0f}};
    auto v = Penguin::Vector3f{0.0f, 0.0f, 1.0f};
    auto v2 = q.apply(v);
    TEST_CHECK_FLOAT_VALUE(v2[0], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[2], -v[2], 0.0001f);
    
    Penguin::Quaternion q2{std::cos(3.14159265f/2.0f), 
            std::sin(3.14159265f/2.0f)*Penguin::Vector3f{0.0f, 1.0f, 0.0f}};
    auto vv = Penguin::Vector3f{0.0f, 1.0f, 0.0f};
    auto vv2 = q2.apply(vv);
    TEST_CHECK_FLOAT_VALUE(vv2[0], vv[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vv2[1], vv[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vv2[2], vv[2], 0.0001f);
    
    Penguin::Quaternion q3{std::cos(3.14159265f/2.0f), 
            std::sin(3.14159265f/2.0f)*Penguin::Vector3f{0.0f, 0.0f, 1.0f}};
    auto vvv = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto vvv2 = q3.apply(vvv);
    TEST_CHECK_FLOAT_VALUE(vvv2[0], -vvv[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[2], 0.0f, 0.0001f);
}