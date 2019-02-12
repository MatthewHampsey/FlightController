#pragma once

#include "rotation.h"
#include "vector3f.h"

namespace FrameDrag {
Rotation ZYXEulerToRotation(float yaw, float pitch, float roll);
Rotation ZYXEulerToRotation(const Vector3f &euler);

Vector3f ZYXEulerToAngularVelocity(const Vector3f &euler,
                                   const Vector3f &euler_derivative);
Vector3f ZYXEulerToAngularVelocity(float yaw, float pitch, float roll,
                                   float yaw_deriv, float pitch_deriv,
                                   float roll_deriv);
Vector3f AngularVelocityToZYXEulerDerivatives(const Vector3f &euler,
                                   const Vector3f &angular_velocity);
}