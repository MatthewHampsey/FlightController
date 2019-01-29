#pragma once

#include <memory>
#include <initializer_list>

namespace Penguin{
class Vector3f{

  struct impl;
  struct impl_deleter { void operator()(impl*) const; };

  public:
  Vector3f();
  Vector3f(float x, float y, float z);
  Vector3f(const std::initializer_list<float>& l);
  Vector3f(const Vector3f& other);
  Vector3f(Vector3f&&) = default;

  float& operator[](size_t i);
  const float& operator[](size_t i) const;

  bool operator==(const Vector3f& v) const;

  Vector3f operator-(const Vector3f& v) const;
  Vector3f operator+(const Vector3f& v) const;


  friend Vector3f operator*(float x, const Vector3f& v);
  friend std::ostream& operator<<(std::ostream& os, const Vector3f& v);
  
  private:
  std::unique_ptr<impl, impl_deleter> _impl;//repr_type _vec;
};
}