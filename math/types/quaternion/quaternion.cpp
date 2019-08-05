#include "quaternion.h"
#include <cmath>
#include <iostream>

namespace FrameDrag {
Quaternion::Quaternion() : _imag{0.0f, 0.0f, 0.0f}, _real{0.0f} {}

Quaternion::Quaternion(float w, Vector3f&& v) : _imag{v}, _real{w} {}

Quaternion::Quaternion(float w, const Vector3f& v) : _imag{v}, _real{w} {}

Quaternion::Quaternion(float q1, float q2, float q3, float q4)
    : _imag{q2, q3, q4}, _real{q1} {}

Vector3f Quaternion::apply(const Vector3f& v) const {
  // (0, v') = qvq∗
  //         = (re, im)(0, v)(re, -im)
  //         = (-im⋅v, re*v + im×v)(re, -im)
  //         = (-(im⋅v)*re + (re*v + im×v)⋅im,
  //            (im⋅v)im + (re*v + im×v)re + (re*v + im×v)×(-im))
  //         = (-(im⋅v)*re + (v⋅im)*re + (im×v)⋅im),
  //            im×(im×v) + (im⋅im)v + (re*re)v + re(im×v) + re(im×v) +
  //            im×(im×v))
  //         = (0, (re*re + im⋅im)v + 2*re(im×v) + 2*im×(im×v))
  //         = (0, |q|^2 v + 2*re(im×v) + 2*im×(im×v))

  auto q = 2.0f * _imag.cross(v);
  return (_real * _real + _imag.innerProduct(_imag)) * v + _real * q +
         _imag.cross(q);
}

Quaternion& Quaternion::operator+=(const Quaternion& q) {
  _real += q._real;
  _imag += q._imag;
  return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& q) {
  _real -= q._real;
  _imag -= q._imag;
  return *this;
}

Quaternion& Quaternion::operator*=(float f) {
  _real *= f;
  _imag *= f;
  return *this;
}

Quaternion& Quaternion::operator/=(float f) {
  _real /= f;
  _imag /= f;
  return *this;
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
  return Quaternion{_real * q._real - _imag.innerProduct(q._imag),
                    _real * q._imag + q._real * _imag + _imag.cross(q._imag)};
}

Quaternion Quaternion::operator+(const Quaternion& q) const {
  return Quaternion{_real + q._real, _imag + q._imag};
}

Quaternion Quaternion::operator-(const Quaternion& q) const {
  return Quaternion{_real - q._real, _imag - q._imag};
}

Quaternion operator*(float x, const Quaternion& q) {
  return Quaternion{x * q._real, x * q._imag};
}

Quaternion operator*(const Quaternion& q, float x) {
  return x * q;
}

Quaternion operator/(const Quaternion& q, float x) {
  return Quaternion{q._real / x, q._imag / x};
}

Quaternion Quaternion::conjugate() const {
  return Quaternion{_real, -1.0f * _imag};
}

Quaternion Quaternion::inverse() const {
  return 1.0f / (_real * _real + _imag.innerProduct(_imag)) * conjugate();
}

float Quaternion::norm() const {
  return std::sqrt(_real * _real + _imag.innerProduct(_imag));
}

float& Quaternion::re() {
  return _real;
}

const float& Quaternion::re() const {
  return _real;
}

Vector3f& Quaternion::im() {
  return _imag;
}

const Vector3f& Quaternion::im() const {
  return _imag;
}

std::ostream& operator<<(std::ostream& out, const Quaternion& q) {
  out << q.re() << " "
      << " " << q.im();
  return out;
}
}
