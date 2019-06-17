#include "prop_controller_x.h"

namespace FrameDrag {
PropellerControllerX::PropellerControllerX(float x, float y, float k_f, float k_t)
    : _k_f(k_f)
    , _control_to_prop_velocity(
          constructControlToPropVelocityTransform(x, y, k_f, k_t))
    , _prop_velocity_to_control(
          constructPropVelocityToControlTransform(x, y, k_f, k_t))
{
}

Matrix4f PropellerControllerX::constructControlToPropVelocityTransform(
    float x, float y, float k_f, float k_t) const
{
    //float moment_factor = 1.0f / (2.0f * length * k_f);
    //float torque_factor = 1.0f / (4.0f * k_t);
    //float thrust_factor = 1.0f / (4.0f * k_f);
    //return Matrix4f{
    //    -moment_factor, 0.0f, torque_factor, thrust_factor,
    //    0.0f, -moment_factor, -torque_factor, thrust_factor,
    //    moment_factor, 0.0f, torque_factor, thrust_factor,
    //    0.0f, moment_factor, -torque_factor, thrust_factor
    //};
    return constructPropVelocityToControlTransform(x, y, k_f, k_t).inverse();
    
}

Matrix4f PropellerControllerX::constructPropVelocityToControlTransform(
    float x, float y, float k_f, float k_t) const
{
    return Matrix4f{ y * k_f, -y * k_f, -y * k_f, y * k_f,
        -x * k_f, -x * k_f, x * k_f, x * k_f,
        k_t, -k_t, k_t, -k_t,
        k_f, k_f, k_f, k_f };
}

float PropellerControllerX::getCurrentThrust()
{
    return _k_f * (_prop_speed[0] + _prop_speed[1] + _prop_speed[2] + _prop_speed[3]);
}

Vector3f PropellerControllerX::getCurrentTorque()
{
    auto control = _prop_velocity_to_control * _prop_speed;
    return Vector3f{ control[0], control[1], control[2] };
}

void PropellerControllerX::applyControlTargets(float target_thrust,
    const Vector3f& target_torque)
{
    Vector4f control{ target_torque[0], target_torque[1], target_torque[2],
        target_thrust };

    _prop_speed = _control_to_prop_velocity * control;
}

Vector4f PropellerControllerX::getCurrentPropSpeed() { return _prop_speed; }
}
