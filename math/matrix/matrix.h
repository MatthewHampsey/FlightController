#pragma once
#include "vector3f.h"
#include "matrix.h"
#include <initializer_list>

namespace FrameDrag {
class Matrix3f {
  struct impl;
  struct impl_deleter {
    void operator()(impl *) const;
  };

public:
  Matrix3f(const std::initializer_list<float> &v);

  friend Vector3f operator*(const Matrix3f &m, const Vector3f &v);

private:
  std::unique_ptr<impl, impl_deleter> _impl;
};
}