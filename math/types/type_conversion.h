#pragma once

#include "matrix.h"
#include "quaternion.h"
#include "vector4f.h"

namespace FrameDrag{

Matrix3f quaternionToRotationMatrix(const Quaternion& q);
Quaternion ZYXEulerToQuaternion(const Vector3f& v);
Vector3f QuaternionToZYXEuler(const Quaternion& q);
Quaternion toQuaternion(const Vector4f& vec);

Matrix3f vectorToSkewSymmetricMatrix(const Vector3f& v);
}
