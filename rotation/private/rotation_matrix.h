#pragma once
#include "vector3f.h"
#include <Eigen/Dense>
#include <initializer_list>

namespace FrameDrag {
class RotationMatrix {
public:
  RotationMatrix(std::initializer_list<float> v);

  Vector3f apply(const Vector3f &v) const;

private:
  Eigen::Matrix3f _mat;
};
}