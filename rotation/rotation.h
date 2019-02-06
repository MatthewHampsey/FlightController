#pragma once
#include "vector3f.h"
#include <initializer_list>
#include <iostream>
#include <memory>

namespace FrameDrag {

class Rotation {
private:
  struct RotationPrivateBase {
    virtual ~RotationPrivateBase() = default;
    virtual Vector3f apply(const Vector3f &other) = 0;
  };

  template <typename T> struct RotationPrivate : RotationPrivateBase {
    T m_t;
    RotationPrivate(T &&t) : m_t(std::move(t)) {}
    Vector3f apply(const Vector3f &other) override { return m_t.apply(other); }
  };

public:
  template <typename T> Rotation(T &&t) {
    representation = std::make_unique<RotationPrivate<T>>(std::move(t));
  }
  // public:

  // Rotation(const Rotation&) = delete;
  // friend Rotation fromAxisAngle(float angle, Eigen::Vector3f);

  template <typename T> T apply(const T &v) { return representation->apply(v); }

private:
  std::unique_ptr<RotationPrivateBase> representation;
  // representation of rotation
  // matrix or quat
};

Rotation fromAngleAxis(float angle, const Vector3f &v);
Rotation fromAngleAxis(float angle, Vector3f &&v);
Rotation fromMatrix(const std::initializer_list<float> &l);
}