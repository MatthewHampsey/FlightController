#pragma once

#include "matrix.h"
#include "vector3f.h"
#include "vector4f.h"

namespace FrameDrag {
class PropellerController {
public:
    PropellerController(float length, float k_f, float k_t);
    float getCurrentThrust();
    Vector3f getCurrentTorque();
    void applyControlTargets(float thrust, const Vector3f& target_torque);
    Vector4f getCurrentPropSpeed();

private:
    Matrix4f constructControlToPropVelocityTransform(float length, float k_f,
        float k_t) const;
    Matrix4f constructPropVelocityToControlTransform(float length, float k_f,
        float k_t) const;
    float _k_f;
    Matrix4f _control_to_prop_velocity;
    Matrix4f _prop_velocity_to_control;

    //       ac
    //     1 o
    //   0   |             ^
    // c o---+---o c      x|
    //       |   2     y   |
    //       o 3       <---
    //      ac
    Vector4f _prop_speed{ 0.0f, 0.0f, 0.0f, 0.0f };
};
}