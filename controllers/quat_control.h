#pragma once

#include "matrix.h"
#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {
// Based on "Geometric tracking control of a quadrotor UAV on SE(3)" by Taeyoung
// Lee, Melvin Leok, and N Harris McClamroch,
// which I've translated to the quaternion equivalent
class QuaternionController {
public:
    QuaternionController(const Matrix3f& moment_of_inertia);
    Vector3f getControlVector(const Quaternion& q_orientation,
        const Vector3f& angular_velocity,
        const Quaternion& q_target,
        const Vector3f& target_angular_velocity);

    void setParameters(float K_omega_x, float K_omega_y, float K_omega_z,
        float K_R_x, float K_R_y, float K_R_z);

    void setParameters(float K_omega, float K_R);

    Matrix3f K_omega();

    Matrix3f K_R();

private:
    Matrix3f _I;
    Matrix3f _K_omega;
    Matrix3f _K_R;
};
}
