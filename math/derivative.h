#pragma once

namespace FrameDrag {

template <typename T>
T numerical_derivative(const T &v1, const T &v2, float time_delta) {
  return (1.0f / time_delta) * (v2 - v1);
}
}
