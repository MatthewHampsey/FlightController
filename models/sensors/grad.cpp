#include "grad.h"

#include "matrix.h"
#include "quaternion.h"
#include "vector3f.h"
#include "vector4f.h"

namespace FrameDrag {

Quaternion grad(const Quaternion& sensor_frame_to_world,
    const Vector3f& v,
    const Vector3f& measurement)
{
    const float q0 = sensor_frame_to_world.re();
    const float q1 = sensor_frame_to_world.im()[0];
    const float q2 = sensor_frame_to_world.im()[1];
    const float q3 = sensor_frame_to_world.im()[2];

    // Matrix4f jacobian{2*(q3*v[1] - q2*v[2]), 2*(q2*v[1] + q3*v[2]), 2*(q1*v[1] - q0*v[2]) - 4*q2*v[0], 2*(q1*v[2] + q0*v[1]) -4*q3*v[0],
    //                   2*(q1*v[2] - q3*v[0]), 2*(q2*v[0] + q0*v[2]) - 4*q1*v[1], 2*(q3*v[2] + q1*v[0]), 2*(q2*v[2] - q0*v[0]) - 4*q3*v[1],
    //                   2*(q2*v[0] - q1*v[1]), 2*(q3*v[0] - q0*v[1]) - 4*q1*v[2], 2*(q3*v[1] + q0*v[0]) - 4*q2*v[2], 2*(q1*v[0] + q2*v[1]),
    //                   0.0f, 0.0f, 0.0f, 0.0f};
    Matrix4f jacobian_transpose{
        2 * (q3 * v[1] - q2 * v[2]), 2 * (q1 * v[2] - q3 * v[0]), 2 * (q2 * v[0] - q1 * v[1]), 0.0f,
        2 * (q2 * v[1] + q3 * v[2]), 2 * (q2 * v[0] + q0 * v[2]) - 4 * q1 * v[1], 2 * (q3 * v[0] - q0 * v[1]) - 4 * q1 * v[2], 0.0f,
        2 * (q1 * v[1] - q0 * v[2]) - 4 * q2 * v[0], 2 * (q3 * v[2] + q1 * v[0]), 2 * (q3 * v[1] + q0 * v[0]) - 4 * q2 * v[2], 0.0f,
        2 * (q1 * v[2] + q0 * v[1]) - 4 * q3 * v[0], 2 * (q2 * v[2] - q0 * v[0]) - 4 * q3 * v[1], 2 * (q1 * v[0] + q2 * v[1]), 0.0f
    };
    auto temp = sensor_frame_to_world.conjugate().apply(v) - measurement;
    Vector4f temp_q{ temp[0], temp[1], temp[2], 0.0f };
    Vector4f result = jacobian_transpose * temp_q;

    return Quaternion{ result[0], result[1], result[2], result[3] };
}
}