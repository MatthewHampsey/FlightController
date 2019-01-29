#include "rotation_matrix.h"

namespace Penguin{
  RotationMatrix::RotationMatrix(std::initializer_list<float> v)
  : _mat{Eigen::Matrix3f{v.begin()}.transpose()}{
  }
  
  Vector3f RotationMatrix::apply(const Vector3f& v) const{
    Eigen::Vector3f vv{v[0], v[1], v[2]};
    auto vec = _mat*vv;
    return Vector3f(vec[0], vec[1], vec[2]);
  }
}