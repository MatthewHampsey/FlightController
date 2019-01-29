#pragma once
#include <initializer_list>
#include <Eigen/Dense>
#include "vector3f.h"

namespace Penguin{
class RotationMatrix{
  public:
  RotationMatrix(std::initializer_list<float> v);
  
  Vector3f apply(const Vector3f& v) const;

  private:
  Eigen::Matrix3f _mat;
};
}