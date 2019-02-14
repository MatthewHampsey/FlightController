#include "vector3f.h"
#include <Eigen/Dense>

namespace FrameDrag {

struct Vector3f::impl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  impl(float x, float y, float z) : _vec{Eigen::Vector3f{x, y, z}} {}
  impl(const std::initializer_list<float> &l) : _vec{l.begin()} {}
  impl(const impl &other): _vec{other._vec} {}
  inline float &operator[](size_t i) { return _vec[i]; }
  inline const float &operator[](size_t i) const { return _vec[i]; }
  Eigen::Vector3f _vec;
};

Vector3f::Vector3f() : _impl{new impl{0.0f, 0.0f, 0.0f}} {}
Vector3f::Vector3f(float x, float y, float z) : _impl{new impl{x, y, z}} {}

Vector3f::Vector3f(const std::initializer_list<float> &l)
    : _impl{new impl{l}} {}

Vector3f::Vector3f(const Vector3f &other)
 : _impl{new impl{*other._impl}}
 {}

Vector3f& Vector3f::operator=(const Vector3f& other){
  Vector3f v{other};
  return *this = std::move(v); 
}

void Vector3f::impl_deleter::operator()(Vector3f::impl *ptr) const {
  delete ptr;
}

float &Vector3f::operator[](size_t i) { return _impl->operator[](i); }
const float &Vector3f::operator[](size_t i) const {
  return _impl->operator[](i);
}

bool Vector3f::operator==(const Vector3f &v) const {
  return (_impl->_vec == v._impl->_vec);
}

Vector3f Vector3f::operator-() const {
  Vector3f vv;
  vv._impl->_vec = -_impl->_vec;
  return vv;
}

Vector3f Vector3f::operator-(const Vector3f &v) const {
  Vector3f vv;
  vv._impl->_vec = _impl->_vec - v._impl->_vec;
  return vv;
}

Vector3f& Vector3f::operator-=(const Vector3f &v) {
  _impl->_vec -= v._impl->_vec;
  return *this;
}

Vector3f Vector3f::operator+(const Vector3f &v) const {
  Vector3f vv;
  vv._impl->_vec = _impl->_vec + v._impl->_vec;
  return vv;
}

Vector3f& Vector3f::operator+=(const Vector3f &v) {
  _impl->_vec += v._impl->_vec;
  return *this;
}


Vector3f Vector3f::cross(const Vector3f &v) const{
  Vector3f vv;
  vv._impl->_vec = _impl->_vec.cross(v._impl->_vec);
  return vv;
}

Vector3f operator*(float x, const Vector3f &v) {
  Vector3f v2;
  v2._impl->_vec = x * (v._impl->_vec);
  return v2;
}

Vector3f operator*(const Vector3f &v, float x) { return x * v; }

Vector3f operator/(const Vector3f &v, float x){
  Vector3f v2;
  v2._impl->_vec = v._impl->_vec / x;
  return v2;
}

std::ostream &operator<<(std::ostream &os, const Vector3f &v) {
  os << v[0] << " " << v[1] << " " << v[2];
  return os;
}
}