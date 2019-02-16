#pragma once
#include "vector3f.h"
#include <Eigen/Geometry>

namespace FrameDrag {
class Quaternion {
public:
  Quaternion();

  Quaternion(float w, Vector3f &&v);

  Quaternion(float w, const Vector3f &v);

  Quaternion(float q1, float q2, float q3, float q4);

  Vector3f apply(const Vector3f &v);

  Quaternion inverse();

private:
  Eigen::Quaternionf _quat;
};
}