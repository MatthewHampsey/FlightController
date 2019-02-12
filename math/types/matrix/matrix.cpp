#include "matrix.h"
#include "vector3f.h"
#include <Eigen/Dense>

namespace FrameDrag {

struct Vector3f::impl {
  Eigen::Vector3f _vec;
};

struct Matrix3f::impl {
  impl(const std::initializer_list<float> &v)
      : _mat{Eigen::Matrix3f{v.begin()}.transpose()} {}
  impl(const impl& other): _mat{other._mat} {}
  Eigen::Matrix3f _mat;
};

Matrix3f::Matrix3f()
    : _impl{new impl{0.0f, 0.0f, 0.0f, 
                     0.0f, 0.0f, 0.0f, 
                     0.0f, 0.0f, 0.0f}} {}

Matrix3f::Matrix3f(const std::initializer_list<float> &v)
    : _impl{new impl{v}} {}

Matrix3f::Matrix3f(const Matrix3f& other): _impl{new impl{*other._impl}}{}

Matrix3f& Matrix3f::operator=(const Matrix3f& other){
  Matrix3f m{other};
  return *this = std::move(m); 
}

void Matrix3f::impl_deleter::operator()(Matrix3f::impl *ptr) const {
  delete ptr;
}

float& Matrix3f::operator()(size_t i, size_t j){
  return _impl->_mat(i, j);
}

Matrix3f Matrix3f::inverse(){
  Matrix3f inverse_matrix;
  inverse_matrix._impl->_mat = _impl->_mat.inverse();
  return inverse_matrix;
}

Vector3f operator*(const Matrix3f &m, const Vector3f &v) {
  Vector3f vv;
  vv._impl->_vec = m._impl->_mat*v._impl->_vec;
  return vv;
}

Matrix3f operator*(const Matrix3f &l, const Matrix3f &r) {
  Matrix3f mat;
  mat._impl->_mat = l._impl->_mat*r._impl->_mat;
  return mat;
}

Vector3f Matrix3f::apply(const Vector3f &v){
  return *this * v;
}
}