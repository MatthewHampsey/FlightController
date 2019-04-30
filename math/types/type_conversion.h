#pragma once

#include "matrix.h"
#include "quaternion.h"

namespace FrameDrag{

Matrix3f quaternionToRotationMatrix(const Quaternion& q);

Matrix3f vectorToSkewSymmetricMatrix(const Vector3f& v);
}