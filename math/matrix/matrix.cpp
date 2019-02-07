#include "matrix.h"
#include <Eigen/Dense>

namespace FrameDrag {

struct Matrix3f::impl {
  impl(const std::initializer_list<float> &v)
      : _mat{Eigen::Matrix3f{v.begin()}.transpose()} {}
  Eigen::Matrix3f _mat;
};

Matrix3f::Matrix3f(const std::initializer_list<float> &v)
    : _impl{new impl{v}} {}

void Matrix3f::impl_deleter::operator()(Matrix3f::impl *ptr) const {
  delete ptr;
}

Vector3f operator*(const Matrix3f &m, const Vector3f &v) {
  Eigen::Vector3f vv{v[0], v[1], v[2]};
  auto vec = m._impl->_mat * vv;
  return Vector3f(vec[0], vec[1], vec[2]);
}
}