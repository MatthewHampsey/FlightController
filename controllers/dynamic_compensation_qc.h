#pragma once

#include "quadcopter_controller.h"
#include "vector3f.h"
#include "matrix.h"
#include "euler.h"

namespace FrameDrag{
    // Based on "Attitude Stabilization for a Quadrotor 
    // Helicopter Using a PD Controller" by Long Chen and Gang Wang
class PDDynamic : public QuadcopterController{
    public:
    PDDynamic(const Matrix3f &moment_of_inertia);
    Vector3f getControlVector(const Vector3f &euler_angles,
                              const Vector3f &euler_derivatives,
                              const Vector3f &target_euler_angles,
                              const Vector3f &target_euler_derivatives);

    Matrix3f _I;
    Matrix3f _K_p;
    Matrix3f _K_d;

};
}