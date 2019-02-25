#pragma once

#include "euler.h"
#include "matrix.h"
#include "quadcopter_controller.h"
#include "vector3f.h"

namespace FrameDrag {
// Based on "Attitude Stabilization for a Quadrotor
// Helicopter Using a PD Controller" by Long Chen and Gang Wang
class PDDynamic : public QuadcopterController {
public:
    PDDynamic(const Matrix3f& moment_of_inertia);
    Vector3f getControlVector(const Vector3f& euler_angles,
        const Vector3f& euler_derivatives,
        const Vector3f& target_euler_angles,
        const Vector3f& target_euler_derivatives);
    void setParameters(float damping_factor_x, float natural_frequency_x,
        float damping_factor_y, float natural_frequency_y,
        float damping_factor_z, float natural_frequency_z);
    void setParameters(float damping_factor, float natural_frequency);

    Matrix3f K_p();
    Matrix3f K_d();

private:
    Matrix3f _I;
    Matrix3f _K_p;
    Matrix3f _K_d;
};
}