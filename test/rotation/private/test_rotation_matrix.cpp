#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "RotationMatrixTest"
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include "rotation_matrix.h"
#include "test_util.h"
#include <cmath>

BOOST_AUTO_TEST_CASE(test_identity_matrix) {
    Penguin::RotationMatrix r{1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f};

    auto v = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto v2 = r.apply(v);
    BOOST_CHECK_EQUAL(v, v2);
    
    auto vv = Penguin::Vector3f{0.0f, 1.0f, 0.0f};
    auto vv2 = r.apply(vv);
    BOOST_CHECK_EQUAL(vv, vv2);

    auto vvv = Penguin::Vector3f{0.0f, 0.0f, 1.0f};
    auto vvv2 = r.apply(vvv);
    BOOST_CHECK_EQUAL(vvv, vvv2);
}

BOOST_AUTO_TEST_CASE(test_quarter_pi_y_axis_matrix) {
    const float quarter_pi = 3.14159265f/4.0f;
    Penguin::RotationMatrix r{std::cos(quarter_pi), 0.0f, std::sin(quarter_pi),
                              0.0f,                 1.0f, 0.0f,
                              -std::sin(quarter_pi), 0.0f, std::cos(quarter_pi)};
                              

    const float forty_five_degrees = 1.0f/std::sqrt(2.0f);

    auto v = Penguin::Vector3f{1.0f, 0.0f, 0.0f};
    auto v2 = r.apply(v);
    TEST_CHECK_FLOAT_VALUE(v2[0], forty_five_degrees, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(v2[2], -forty_five_degrees, 0.0001f);
    
    auto vv = Penguin::Vector3f{0.0f, 1.0f, 0.0f};
    auto vv2 = r.apply(vv);
    BOOST_CHECK_EQUAL(vv, vv2);

    auto vvv = Penguin::Vector3f{0.0f, 0.0f, 1.0f};
    auto vvv2 = r.apply(vvv);
    TEST_CHECK_FLOAT_VALUE(vvv2[0], forty_five_degrees, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[1], 0.0f, 0.0001f);
    TEST_CHECK_FLOAT_VALUE(vvv2[2], forty_five_degrees, 0.0001f);
}