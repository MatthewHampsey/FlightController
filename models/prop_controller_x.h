#pragma once

#include "matrix.h"
#include "vector3f.h"
#include "vector4f.h"

namespace FrameDrag {
class PropellerControllerX {
public:
    PropellerControllerX(float x, float y, float k_f, float k_t);
    float getCurrentThrust();
    Vector3f getCurrentTorque();
    void applyControlTargets(float thrust, const Vector3f& target_torque);
    Vector4f getCurrentPropSpeed();

private:
    Matrix4f constructControlToPropVelocityTransform(float x, float y, float k_f,
        float k_t) const;
    Matrix4f constructPropVelocityToControlTransform(float z, float y, float k_f,
        float k_t) const;
    float _k_f;
    Matrix4f _control_to_prop_velocity;
    Matrix4f _prop_velocity_to_control;

    //
    //   c   ac
    // 0 o   o 1           ^
    //    \ /             x|
    //     +           y   |
    //    / \          <---
    // 3 o   o 2
    //  ac   c 
    //
    Vector4f _prop_speed{ 0.0f, 0.0f, 0.0f, 0.0f };
};
}
