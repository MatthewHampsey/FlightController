#include "quaternion.h"

namespace FrameDrag {
Quaternion::Quaternion()
    : _quat{ 0.0f, 0.0f, 0.0f, 0.0f }
{
}

Quaternion::Quaternion(float w, Vector3f&& v)
    : _quat{ w, v[0], v[1], v[2] }
{
}

Quaternion::Quaternion(float w, const Vector3f& v)
    : _quat{ w, v[0], v[1], v[2] }
{
}

Quaternion::Quaternion(float q1, float q2, float q3, float q4)
    : _quat{ q1, q2, q3, q4 }
{
}

Vector3f Quaternion::apply(const Vector3f& v)
{
    Eigen::Quaternionf q_v{ 0.0f, v[0], v[1], v[2] };
    auto rotated = _quat * q_v * _quat.inverse();
    return Vector3f{ rotated.vec()[0], rotated.vec()[1], rotated.vec()[2] };
}

Quaternion Quaternion::inverse()
{
    Quaternion q;
    q._quat = _quat.inverse();
    return q;
}
}