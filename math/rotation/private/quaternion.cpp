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

Vector3f Quaternion::apply(const Vector3f& v) const
{
    Eigen::Quaternionf q_v{ 0.0f, v[0], v[1], v[2] };
    auto rotated = _quat * q_v * _quat.inverse();
    return Vector3f{ rotated.vec()[0], rotated.vec()[1], rotated.vec()[2] };
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
    Quaternion qq;
    qq._quat = _quat * q._quat;
    return qq;
}

Quaternion Quaternion::operator+(const Quaternion& q) const
{
    Quaternion qq;
    qq._quat.w() = _quat.w() + q._quat.w();
    qq._quat.vec() = _quat.vec() + q._quat.vec();
    return qq;
}

Quaternion Quaternion::operator-(const Quaternion& q) const
{
    Quaternion qq;
    qq._quat.w() = _quat.w() - q._quat.w();
    qq._quat.vec() = _quat.vec() - q._quat.vec();
    return qq;
}

Quaternion operator*(float x, const Quaternion& q)
{
    auto v = q._quat.vec();
    return Quaternion{ x * q._quat.w(), x * v[0], x * v[1], x * v[2] };
}

Quaternion operator*(const Quaternion& q, float x)
{
    return x * q;
}

Quaternion operator/(const Quaternion& q, float x)
{
    return (1.0 / x) * q;
}

Quaternion Quaternion::inverse() const
{
    Quaternion q;
    q._quat = _quat.inverse();
    return q;
}

float Quaternion::re() const
{
    return _quat.w();
}

Vector3f Quaternion::im() const
{
    auto im = _quat.vec();
    return Vector3f{ im[0], im[1], im[2] };
}
}