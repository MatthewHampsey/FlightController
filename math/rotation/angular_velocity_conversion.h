#pragma once

#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

Vector3f QuaternionToBodyFrameAngularVelocity(const Quaternion& q,
    const Quaternion& q_derivative);

Vector3f QuaternionAndTimeToBodyFrameAngularVelocity(const Quaternion& q1, const Quaternion& q2, float time_diff); 

Quaternion BodyFrameAngularVelocityToQuaternionDerivative(
    const Quaternion& q, const Vector3f& angular_velocity);
}
