#include "type_conversion.h"
#include <cmath>

namespace FrameDrag{

Matrix3f quaternionToRotationMatrix(const Quaternion& q)
{
    auto skew = vectorToSkewSymmetricMatrix(q.im());
    auto id = Matrix3f{1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f};
    return id + 2*q.re()*skew + 2*skew*skew;
}

Quaternion ZYXEulerToQuaternion(const Vector3f& v)
{
    // yaw quat:   [cos(yaw/2)]
    //            [          ]
    //            [          ]
    //            [sin(yaw/2)]
    // pitch quat: [cos(pitch/2)]
    //            [          ]
    //            [sin(yaw/2)]
    //            [          ]
    // roll quat:  [cos(roll/2)]
    //            [sin(roll/2)]
    //            [           ]
    //            [           ]
    // ik = -j
    // final quat: [cos(yaw/2)][cos(pitch/2)][cos(roll/2)]  [cos(yaw/2)*cos(pitch/2) ][cos(roll/2)]
    //            [          ][            ][sin(roll/2)] = [-sin(yaw/2)*sin(pitch/2)][sin(roll/2)]
    //            [          ][ sin(pitch/2)[           ]   [cos(yaw/2)*sin(pitch/2) ][           ]
    //            [sin(yaw/2)][            ][           ]   [sin(yaw/2)*cos(pitch/2) ][           ]
    //
    // final quat: [cos(yaw/2)*cos(pitch/2) ][cos(roll/2)]   [cos(yaw/2)*cos(pitch/2)*cos(roll/2) + sin(yaw/2)*sin(pitch/2)*sin(roll/2)]
    //             [-sin(yaw/2)*sin(pitch/2)][sin(roll/2)] = [cos(yaw/2)*cos(pitch/2)*sin(roll/2) - sin(yaw/2)*sin(pitch/2)*cos(roll/2)]
    //             [cos(yaw/2)*sin(pitch/2) ][           ]   [cos(yaw/2)*sin(pitch/2)*cos(roll/2) + sin(yaw/2)*cos(pitch/2)*sin(roll/2)]
    //             [sin(yaw/2)*cos(pitch/2) ][           ]   [sin(yaw/2)*cos(pitch/2)*cos(roll/2) - cos(yaw/2)*sin(pitch/2)*sin(roll/2)]
    //
    const auto y2 = v[2] / 2.0f;
    const auto p2 = v[1] / 2.0f;
    const auto r2 = v[0] / 2.0f;
    const auto cosy2 = std::cos(y2);
    const auto siny2 = std::sin(y2);
    const auto cosp2 = std::cos(p2);
    const auto sinp2 = std::sin(p2);
    const auto cosr2 = std::cos(r2);
    const auto sinr2 = std::sin(r2);
    return Quaternion(cosy2 * cosp2 * cosr2 + siny2 * sinp2 * sinr2,
         cosy2 * cosp2 * sinr2 - siny2 * sinp2 * cosr2,
         cosy2 * sinp2 * cosr2 + siny2 * cosp2 * sinr2,
         siny2 * cosp2 * cosr2 - cosy2 * sinp2 * sinr2);
}

Vector3f QuaternionToZYXEuler(const Quaternion& q)
{
  //roll (x-axis rotation)
  auto re = q.re();
  auto v = q.im();
  Vector3f euler;
  float sinr_cosp = +2.0 * (re * v[0] + v[1] * v[2]);
  float cosr_cosp = +1.0 - 2.0 * (v[0] * v[0] + v[1] * v[1]);
  euler[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = +2.0 * (re * v[1] - v[2] * v[0]);
  if (fabs(sinp) >= 1) euler[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else euler[1] = asin(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = +2.0 * (re * v[2] + v[0] * v[1]);
  float cosy_cosp = +1.0 - 2.0 * (v[1] * v[1] + v[2] * v[2]);  
  euler[2] = atan2(siny_cosp, cosy_cosp);

  return euler;
}


Matrix3f vectorToSkewSymmetricMatrix(const Vector3f& v){
    return Matrix3f{0.0f, -v[2], v[1],
                    v[2], 0.0f, -v[0],
                    -v[1], v[0], 0.0f};
}
}
