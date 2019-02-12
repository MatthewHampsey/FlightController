#include "rotation.h"
#include "quaternion.h"
#include "rotation_matrix.h"

namespace FrameDrag {

Rotation Rotation::inverse(){
  return Rotation{_representation->inverse()};
}

Rotation fromAngleAxis(float angle, const Vector3f &v) {
  return Rotation{
      Quaternion(std::cos(angle / 2.0f), std::sin(angle / 2.0f) * v)};
}

Rotation fromAngleAxis(float angle, Vector3f &&v) {
  return Rotation{
      Quaternion(std::cos(angle / 2.0f), std::sin(angle / 2.0f) * v)};
}

Rotation fromMatrix(const std::initializer_list<float> &l) {
  return Rotation{RotationMatrix{l}};
}
}