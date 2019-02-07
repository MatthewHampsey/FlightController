#include "matrix.h"
#include "rotation.h"
#include <cmath>

namespace FrameDrag {

namespace {
inline Matrix3f eulerToAngularVelocityConversionMatrix(const Vector3f &euler) {
  // Derivation:
  // R = (R_z)(R_y)(R_x)
  // dR/dt = (dR_z/dt)(R_y)(R_x) +
  //		   (R_z)(dR_y/dt)(R_x) +
  //		   (R_z)(R_y)(dR_x/dt)
  //
  // S(ω)R = dyaw/dt*S(z)(R_z)(R_y)(R_x) +
  //        (R_z)(dpitch/dt*S(y))(R_y)(R_x) +
  //        (R_z)(R_y)(droll/dt*S(x))(R_x)
  //
  // S(ω)R = dyaw/dt*S(z)R +
  //        (R_z)(dpitch/dt*S(y))(R_z^T)(R_z)(R_y)(R_x) +
  //        (R_z)(R_y)(droll/dt*S(x))((R_z)(R_y))^T(R_z)(R_y)(R_x)
  //
  // S(ω)R = dyaw/dt*S(z)R +
  //         dpitch/dt*S((R_z)y)R +
  //         droll/dt*S((R_z)(R_y)x)R
  //
  // S(ω) =  S(dyaw/dt*z) +
  //         S(dpitch/dt*(R_z)y) +
  //         S(droll/dt*(R_z)(R_y)x)
  //
  // S(ω) =  S(dyaw/dt*z) +
  //         S(dpitch/dt*(R_z)y) +
  //         S(droll/dt*(R_z)(R_y)x)
  // ω    =  dyaw/dt*z +
  //         dpitch/dt*(R_z)y +
  //         droll/dt*(R_z)(R_y)x
  //
  // ω	  =  (    0    )   ( -sin(yaw)*dpitch/dt )    (cos(yaw)*cos(pitch)*droll/dt)
  //         (    0    ) + ( cos(yaw)*dpitch/dt  )  + (sin(yaw)*cos(pitch)*droll/dt)
  //         ( dyaw/dt )   (        0            )    (-sin(pitch)*droll/dt )
  //
  // because R_z * R_y = ( cos(yaw) -sin(yaw)  0 ) ( cos(pitch)  0 sin(pitch) )
  //				             ( sin(yaw) cos(yaw)   0 ) (     0       1    0       )
  //				             (    0        0       1 ) ( -sin(pitch) 0 cos(pitch) )
  //
  // =>           = ( cos(yaw)*cos(pitch)  -sin(yaw)  cos(yaw)*sin(pitch) )
  //				        ( sin(yaw)*cos(pitch)  cos(yaw)   sin(yaw)*sin(pitch) )
  //				        ( -sin(pitch)             0       cos(pitch) )
  //
  // thus ω = ( 0  -sin(yaw)  cos(yaw)*cos(pitch) )( dyaw/dt   )
  //          ( 0   cos(yaw)  sin(yaw)*cos(pitch) )( dpitch/dt )
  //		      ( 1     0      -sin(pitch)          )( droll/dt )
  const float yaw = euler[0];
  const float pitch = euler[1];
  const auto cosy = std::cos(yaw);
  const auto siny = std::sin(yaw);
  const auto cosp = std::cos(pitch);
  const auto sinp = std::sin(pitch);
  return Matrix3f(
      {0.0f, -siny, cosy * cosp, 0.0f, cosy, siny * cosp, 1.0f, 0.0f, -sinp});
}
}

Rotation ZYXEulerToRotation(float yaw, float pitch, float roll) {
  // yaw mat:   [  cos  -sin  0    ]
  //           [ sin   cos   0    ]
  //           [ 0     0     1    ]
  // pitch mat: [ cos   0     sin  ]
  //           [ 0     1     0    ]
  //           [ -sin  0     cos  ]
  // roll mat:  [ 1     0     0    ]
  //           [ 0     cos   -sin ]
  //           [ 0     sin   cos  ]
  // result: [cos(y)cos(p)  -sin(y)  cos(y)sin(p)][1 0   0   ]
  //        [sin(y)cos(p)  cos(y)   sin(y)sin(p)][0 cos -sin]
  //        [-sin(p)       0        cos(p)      ][0 sin cos ]
  //
  // =>     [cos(y)cos(p)   -sin(y)cos(r)+cos(y)sin(p)sin(r) sin(y)sin(r)+cos(y)sin(p)cos(r)    ]
  //        [sin(y)cos(p)   cos(y)cos(r)+sin(y)sin(p)sin(r)     -cos(y)sin(r) +sin(y)sin(p)cos(r) ]
  //        [-sin(p)        cos(p)sin(r)                        cos(p)cos(r) ]

  const auto cosy = std::cos(yaw);
  const auto siny = std::sin(yaw);
  const auto cosp = std::cos(pitch);
  const auto sinp = std::sin(pitch);
  const auto cosr = std::cos(roll);
  const auto sinr = std::sin(roll);
  return fromMatrix({cosy * cosp, -siny * cosr + cosy * sinp * sinr,
                     siny * sinr + cosy * sinp * cosr, siny * cosp,
                     cosy * cosr + siny * sinp * sinr,
                     -cosy * sinr + siny * sinp * cosr, -sinp, cosp * sinr,
                     cosp * cosr});
}

Vector3f ZYXEulerToAngularVelocity(const Vector3f &euler,
                                   const Vector3f &euler_derivative) {
  return eulerToAngularVelocityConversionMatrix(euler) * euler_derivative;
}

Vector3f ZYXEulerToAngularVelocity(float yaw, float pitch, float roll,
                                   float yaw_deriv, float pitch_deriv,
                                   float roll_deriv) {
  return ZYXEulerToAngularVelocity(Vector3f{yaw, pitch, roll},
                                   Vector3f{yaw, pitch, roll});
}
}