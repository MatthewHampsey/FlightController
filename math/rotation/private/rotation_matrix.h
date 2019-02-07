#pragma once
#include "matrix.h"
#include "vector3f.h"
#include <initializer_list>

namespace FrameDrag {
class RotationMatrix : Matrix3f {
public:
  RotationMatrix(const std::initializer_list<float> &v);

  Vector3f apply(const Vector3f &v) const;
};
}