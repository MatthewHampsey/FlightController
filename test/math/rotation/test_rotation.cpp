#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "EulerTest"
#define BOOST_TEST_MAIN
#include "euler.h"
#include "test_util.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

const float forty_five_degrees = 1 / std::sqrt(2);
std::vector<FrameDrag::Rotation (*)(float, float, float)> euler_to_rot_funcs{
    FrameDrag::ZYXEulerToRotationQuaternion,
    FrameDrag::ZYXEulerToRotationMatrix
};

BOOST_AUTO_TEST_CASE(test_yaw)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation yaw_rotation = f(std::atan(1) * 4, 0.0f, 0.0f);
        FrameDrag::Vector3f x_hat{ 1.0f, 0.0f, 0.0f };
        auto x_hat_rot = yaw_rotation.apply(x_hat);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.0001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.0001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f y_hat{ 0.0f, 1.0f, 0.0f };
        auto y_hat_rot = yaw_rotation.apply(y_hat);

        TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f z_hat{ 0.0f, 0.0f, 1.0f };
        auto z_hat_rot = yaw_rotation.apply(z_hat);

        TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], 1.0f, 0.001f);
    }
}

BOOST_AUTO_TEST_CASE(test_pitch)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation pitch_rotation = f(0.0f, std::atan(1) * 4, 0.0f);
        FrameDrag::Vector3f x_hat{ 1.0f, 0.0f, 0.0f };
        auto x_hat_rot = pitch_rotation.apply(x_hat);

        TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f y_hat{ 0.0f, 1.0f, 0.0f };
        auto y_hat_rot = pitch_rotation.apply(y_hat);

        TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], 1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f z_hat{ 0.0f, 0.0f, 1.0f };
        auto z_hat_rot = pitch_rotation.apply(z_hat);

        TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
    }
}

BOOST_AUTO_TEST_CASE(test_roll)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation roll_rotation = f(0.0f, 0.0f, std::atan(1) * 4);
        FrameDrag::Vector3f x_hat{ 1.0f, 0.0f, 0.0f };
        auto x_hat_rot = roll_rotation.apply(x_hat);

        TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], 1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f y_hat{ 0.0f, 1.0f, 0.0f };
        auto y_hat_rot = roll_rotation.apply(y_hat);

        TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f z_hat{ 0.0f, 0.0f, 1.0f };
        auto z_hat_rot = roll_rotation.apply(z_hat);

        TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
    }
}

BOOST_AUTO_TEST_CASE(test_pitch_then_yaw)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation roll_rotation = f(std::atan(1) * 4, std::atan(1), 0.0f);
        FrameDrag::Vector3f x_hat{ 1.0f, 0.0f, 0.0f };
        auto x_hat_rot = roll_rotation.apply(x_hat);

        TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], -forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], -forty_five_degrees, 0.001f);

        FrameDrag::Vector3f y_hat{ 0.0f, 1.0f, 0.0f };
        auto y_hat_rot = roll_rotation.apply(y_hat);

        TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -1.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f z_hat{ 0.0f, 0.0f, 1.0f };
        auto z_hat_rot = roll_rotation.apply(z_hat);

        TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], -forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], forty_five_degrees, 0.001f);
    }
}

BOOST_AUTO_TEST_CASE(test_roll_then_yaw)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation roll_rotation = f(std::atan(1), 0.0f, std::atan(1) * 4);
        FrameDrag::Vector3f x_hat{ 1.0f, 0.0f, 0.0f };
        auto x_hat_rot = roll_rotation.apply(x_hat);

        TEST_CHECK_FLOAT_VALUE(x_hat_rot[0], forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[1], forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(x_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f y_hat{ 0.0f, 1.0f, 0.0f };
        auto y_hat_rot = roll_rotation.apply(y_hat);

        TEST_CHECK_FLOAT_VALUE(y_hat_rot[0], forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[1], -forty_five_degrees, 0.001f);
        TEST_CHECK_FLOAT_VALUE(y_hat_rot[2], 0.0f, 0.001f);

        FrameDrag::Vector3f z_hat{ 0.0f, 0.0f, 1.0f };
        auto z_hat_rot = roll_rotation.apply(z_hat);

        TEST_CHECK_FLOAT_VALUE(z_hat_rot[0], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[1], 0.0f, 0.001f);
        TEST_CHECK_FLOAT_VALUE(z_hat_rot[2], -1.0f, 0.001f);
    }
}

BOOST_AUTO_TEST_CASE(test_velocity_x_axis)
{
    FrameDrag::Vector3f e_ang{ 0.0f, 0.0f, 0.0f };
    FrameDrag::Vector3f e_deriv{ 1.0f, 0.0f, 0.0f };
    auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
    BOOST_CHECK(angular_velocity.isApprox(e_deriv, 0.01f));
}

BOOST_AUTO_TEST_CASE(test_velocity_y_axis)
{
    FrameDrag::Vector3f e_ang{ 0.0f, 0.0f, 0.0f };
    FrameDrag::Vector3f e_deriv{ 0.0f, 1.0f, 0.0f };
    auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
    BOOST_CHECK(angular_velocity.isApprox(e_deriv, 0.01f));
}

BOOST_AUTO_TEST_CASE(test_velocity_z_axis)
{
    FrameDrag::Vector3f e_ang{ 0.0f, 0.0f, 0.0f };
    FrameDrag::Vector3f e_deriv{ 0.0f, 0.0f, 1.0f };
    auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
    BOOST_CHECK(angular_velocity.isApprox(e_deriv, 0.01f));
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_x_axis)
{
    for (auto& f : euler_to_rot_funcs) {
        // rotate by pi/2 around y-axis, then pi/2 around z-axis
        FrameDrag::Vector3f e_ang{ 0.0f, 3.14159265f / 2.0f, 3.14159265f / 2.0f };
        FrameDrag::Vector3f e_deriv{ 1.0f, 0.0f, 0.0f };
        auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
        // ω_b = R^T*ω
        //     = dyaw/dt*(R_x^T)(R_y^T)(R_z^T)z +
        //         dpitch/dt*(R_x^T)(R_y^T)(R_z^T)(R_z)y +
        //         droll/dt*(R_x^T)(R_y^T)(R_z^T)(R_z)(R_y)x
        //     = dyaw/dt*(R_x^T)(R_y^T)z +
        //         dpitch/dt*(R_x^T)y +
        //         droll/dt*x
        FrameDrag::Vector3f angular_velocity2 = e_deriv[0] * FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f } + e_deriv[1] * f(0.0f, 0.0f, e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 1.0f, 0.0f }) + e_deriv[2] * f(0.0f, e_ang[1], e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 0.0f, 1.0f });
        BOOST_CHECK(angular_velocity.isApprox(angular_velocity2, 0.001f));
    }
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_x_axis_2)
{
    for (auto& f : euler_to_rot_funcs) {
        // rotate by pi/2 around y-axis, then pi/2 around z-axis
        FrameDrag::Vector3f e_ang{ 0.0f, 3.14159265f, 3.14159265f / 2.0f };
        FrameDrag::Vector3f e_deriv{ 1.0f, 0.0f, 0.0f };
        auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
        FrameDrag::Vector3f angular_velocity2 = e_deriv[0] * FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f } + e_deriv[1] * f(0.0f, 0.0f, e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 1.0f, 0.0f }) + e_deriv[2] * f(0.0f, e_ang[1], e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 0.0f, 1.0f });
        BOOST_CHECK(angular_velocity.isApprox(angular_velocity2, 0.001f));
    }
}

BOOST_AUTO_TEST_CASE(test_velocity_rotated_y_axis)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Vector3f e_ang{ 3.14159265f / 4.0f, 0.0f, 3.14159265f / 2.0f };
        FrameDrag::Vector3f e_deriv{ 0.0f, 1.0f, 0.0f };
        auto angular_velocity = ZYXEulerToBodyFrameAngularVelocity(e_ang, e_deriv);
        FrameDrag::Vector3f angular_velocity2 = e_deriv[0] * FrameDrag::Vector3f{ 1.0f, 0.0f, 0.0f } + e_deriv[1] * f(0.0f, 0.0f, e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 1.0f, 0.0f }) + e_deriv[2] * f(0.0f, e_ang[1], e_ang[0]).inverse().apply(FrameDrag::Vector3f{ 0.0f, 0.0f, 1.0f });
        BOOST_CHECK(angular_velocity.isApprox(angular_velocity2, 0.001f));
    }
}

BOOST_AUTO_TEST_CASE(test_rot_inverse)
{
    for (auto& f : euler_to_rot_funcs) {
        FrameDrag::Rotation r = f(std::atan(1) * 4, 1.2f, 2.34f);
        FrameDrag::Vector3f v{ 5.0f, 3.2f, 9.1f };
        FrameDrag::Vector3f v2 = r.apply(v);
        FrameDrag::Vector3f v3 = r.inverse().apply(v2);
        BOOST_CHECK(v.isApprox(v3, 0.001f));
    }
}
