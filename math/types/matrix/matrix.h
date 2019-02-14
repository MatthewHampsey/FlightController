#pragma once
#include "matrix.h"
#include <memory>
#include <initializer_list>

namespace FrameDrag {

class Vector3f;

class Matrix3f {
  struct impl;
  struct impl_deleter {
    void operator()(impl *) const;
  };

public:
  Matrix3f();
  Matrix3f(const std::initializer_list<float> &v);
  Matrix3f(const Matrix3f& other);
  Matrix3f& operator=(const Matrix3f& other);
  Matrix3f(Matrix3f&&) = default;
  Matrix3f& operator=(Matrix3f&&) = default;
  Matrix3f inverse();

  Matrix3f operator-() const;
  Matrix3f operator-(const Matrix3f &other) const;
  Matrix3f& operator-=(const Matrix3f &other);
  Matrix3f operator+(const Matrix3f &other) const;
  Matrix3f& operator+=(const Matrix3f &other);
  float& operator()(size_t i, size_t j);
  friend Vector3f operator*(const Matrix3f &m, const Vector3f &v);
  friend Matrix3f operator*(const Matrix3f &l, const Matrix3f &r);
  friend Matrix3f operator*(float x, const Matrix3f &m);  
  friend Matrix3f operator*(const Matrix3f &m, float x);
  friend Matrix3f operator/(const Matrix3f &m, float x);

  Vector3f apply(const Vector3f &v);

private:
  std::unique_ptr<impl, impl_deleter> _impl;
};
}