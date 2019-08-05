#pragma once

#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

Vector3f QuaternionToBodyFrameAngularVelocity(const Quaternion& q,
    const Quaternion& q_derivative);

Quaternion BodyFrameAngularVelocityToQuaternionDerivative(
    const Quaternion& q, const Vector3f& angular_velocity);
}