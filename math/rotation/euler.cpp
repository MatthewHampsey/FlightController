#include "matrix.h"
#include "rotation.h"
#include <cmath>

namespace FrameDrag {

namespace {
    inline Matrix3f
    eulerToBodyFrameAngularVelocityConversionMatrix(const Vector3f& euler)
    {
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
        // ω_b = R^T*ω
        //     = dyaw/dt*(R_x^T)(R_y^T)(R_z^T)z +
        //         dpitch/dt*(R_x^T)(R_y^T)(R_z^T)(R_z)y +
        //         droll/dt*(R_x^T)(R_y^T)(R_z^T)(R_z)(R_y)x
        //     = dyaw/dt*(R_x^T)(R_y^T)z +
        //         dpitch/dt*(R_x^T)y +
        //         droll/dt*x

        // ω_b	 = (droll/dt)   (        0           )   (-sin(pitch)*dyaw/dt)
        //         (   0    ) + (cos(roll)*dpitch/dt ) + (sin(roll)*cos(pitch)*dyaw/dt)
        //         (   0    )   (-sin(roll)*dpitch/dt)   (cos(roll)*cos(pitch)*dyaw/dt )
        //
        // because R_x^T * R_y^T = (1   0            0    ) ( cos(pitch)  0 -sin(pitch) )
        //				                 (0 cos(roll)  sin(roll)) (     0       1     0       )
        //				                 (0 -sin(roll) cos(roll)) ( sin(pitch)  0  cos(pitch) )
        //
        // =>           = ( cos(pitch)              0        -sin(pitch) )
        //				        ( sin(roll)*sin(pitch)  cos(roll)  sin(roll)*cos(pitch) )
        //				        ( cos(roll)*sin(pitch) -sin(roll)  cos(roll)*cos(pitch) )
        //
        // thus ω_b = ( 1     0             -sin(pitch)      )( droll/dt  )
        //            ( 0   cos(roll)   sin(roll)*cos(pitch) )( dpitch/dt )
        //		        ( 0   -sin(roll)  cos(roll)*cos(pitch) )( dyaw/dt   )
        const float roll = euler[0];
        const float pitch = euler[1];
        const auto cosr = std::cos(roll);
        const auto sinr = std::sin(roll);
        const auto cosp = std::cos(pitch);
        const auto sinp = std::sin(pitch);
        return Matrix3f(
            { 1.0f, 0.0f, -sinp,
                0.0f, cosr, sinr * cosp,
                0.0f, -sinr, cosr * cosp });
    }
}

Rotation ZYXEulerToRotationMatrix(float yaw, float pitch, float roll)
{
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
    //        [sin(y)cos(p)   cos(y)cos(r)+sin(y)sin(p)sin(r)     -cos(y)sin(r)+sin(y)sin(p)cos(r) ]
    //        [-sin(p)        cos(p)sin(r)                        cos(p)cos(r) ]

    const auto cosy = std::cos(yaw);
    const auto siny = std::sin(yaw);
    const auto cosp = std::cos(pitch);
    const auto sinp = std::sin(pitch);
    const auto cosr = std::cos(roll);
    const auto sinr = std::sin(roll);
    return fromMatrix({ cosy * cosp, -siny * cosr + cosy * sinp * sinr,
        siny * sinr + cosy * sinp * cosr, siny * cosp,
        cosy * cosr + siny * sinp * sinr,
        -cosy * sinr + siny * sinp * cosr, -sinp, cosp * sinr,
        cosp * cosr });
}

Rotation ZYXEulerToRotationQuaternion(float yaw, float pitch, float roll)
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

    const auto y2 = yaw / 2.0f;
    const auto p2 = pitch / 2.0f;
    const auto r2 = roll / 2.0f;
    const auto cosy2 = std::cos(y2);
    const auto siny2 = std::sin(y2);
    const auto cosp2 = std::cos(p2);
    const auto sinp2 = std::sin(p2);
    const auto cosr2 = std::cos(r2);
    const auto sinr2 = std::sin(r2);
    return fromQuaternion(cosy2 * cosp2 * cosr2 + siny2 * sinp2 * sinr2,
        cosy2 * cosp2 * sinr2 - siny2 * sinp2 * cosr2,
        cosy2 * sinp2 * cosr2 + siny2 * cosp2 * sinr2,
        siny2 * cosp2 * cosr2 - cosy2 * sinp2 * sinr2);
}

Rotation ZYXEulerToRotationQuaternion(const Vector3f& euler)
{
    return ZYXEulerToRotationQuaternion(euler[0], euler[1], euler[2]);
}

Vector3f ZYXEulerToBodyFrameAngularVelocity(const Vector3f& euler,
    const Vector3f& euler_derivative)
{
    return eulerToBodyFrameAngularVelocityConversionMatrix(euler) * euler_derivative;
}

Vector3f ZYXEulerToBodyFrameAngularVelocity(float roll, float pitch, float yaw,
    float roll_deriv, float pitch_deriv,
    float yaw_deriv)
{
    return ZYXEulerToBodyFrameAngularVelocity(
        Vector3f{ roll, pitch, yaw }, Vector3f{ roll_deriv, pitch_deriv, yaw_deriv });
}

Vector3f BodyFrameAngularVelocityToZYXEulerDerivatives(
    const Vector3f& euler, const Vector3f& angular_velocity)
{
    return eulerToBodyFrameAngularVelocityConversionMatrix(euler).inverse() * angular_velocity;
}
}