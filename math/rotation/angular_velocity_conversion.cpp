#include "angular_velocity_conversion.h"
#include "quaternion.h"

namespace FrameDrag {

Vector3f QuaternionToBodyFrameAngularVelocity(const Quaternion& q,
    const Quaternion& q_derivative)
{
    return 2.0f * (q.conjugate() * q_derivative).im();
}

Quaternion
BodyFrameAngularVelocityToQuaternionDerivative(const Quaternion& q,
    const Vector3f& angular_velocity)
{
    return 0.5f * q * Quaternion(0.0f, angular_velocity);
}
}