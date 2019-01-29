#include "quaternion.h"

namespace Penguin{
  Quaternion::Quaternion(float w, Vector3f&& v):_quat{w, v[0], v[1], v[2]}{
  }
  
  Quaternion::Quaternion(float w, const Vector3f& v):_quat{w, v[0], v[1], v[2]}{
  }

  Vector3f Quaternion::apply(const Vector3f& v){
    Eigen::Quaternionf q_v{0.0f, v[0], v[1], v[2]};
    auto rotated = _quat * q_v * _quat.inverse();
    return Vector3f{rotated.vec()[0], rotated.vec()[1], rotated.vec()[2]};
  }


}