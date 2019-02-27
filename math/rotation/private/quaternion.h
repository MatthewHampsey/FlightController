#pragma once
#include "vector3f.h"
#include <Eigen/Geometry>

namespace FrameDrag {
class Quaternion {
public:
    Quaternion();

    Quaternion(float w, Vector3f&& v);

    Quaternion(float w, const Vector3f& v);

    Quaternion(float q1, float q2, float q3, float q4);

    Vector3f apply(const Vector3f& v) const;

    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator+(const Quaternion& q) const;
    Quaternion operator-(const Quaternion& q) const;
    friend Quaternion operator*(float x, const Quaternion& q);
    friend Quaternion operator*(const Quaternion& q, float x);
    friend Quaternion operator/(const Quaternion& q, float x);

    Quaternion inverse() const;

    float re() const;

    Vector3f im() const;

private:
    Eigen::Quaternionf _quat;
};
}