#pragma once
#include "vector3f.h"
#include <Eigen/Geometry>

namespace FrameDrag {
class Quaternion {
public:
  Quaternion(float w, Vector3f &&v);

  Quaternion(float w, const Vector3f &v);

  Vector3f apply(const Vector3f &v);

private:
  Eigen::Quaternionf _quat;
};
}