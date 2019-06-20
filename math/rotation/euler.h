#pragma once

#include "rotation.h"
#include "vector3f.h"

namespace FrameDrag {
Rotation ZYXEulerToRotationMatrix(float roll, float pitch, float yaw);

Rotation ZYXEulerToRotationQuaternion(float roll, float pitch, float yaw);
Rotation ZYXEulerToRotationQuaternion(const Vector3f& euler);

Vector3f ZYXEulerToBodyFrameAngularVelocity(const Vector3f& euler,
    const Vector3f& euler_derivative);
Vector3f ZYXEulerToBodyFrameAngularVelocity(float roll, float pitch, float yaw,
    float roll_deriv, float pitch_deriv,
    float yaw_deriv);
Vector3f
BodyFrameAngularVelocityToZYXEulerDerivatives(const Vector3f& euler,
    const Vector3f& angular_velocity);
}
