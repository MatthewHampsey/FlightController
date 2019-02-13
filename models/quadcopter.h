#pragma once

#include "model.h"
#include "vector3f.h"
#include "matrix.h"
#include "rotation.h"

namespace FrameDrag{
class Quadcopter : Model{
    public:
    Quadcopter();
    Quadcopter(const Vector3f& position,
               const Vector3f& velocity,
               const Vector3f& euler_angles,
               const Vector3f& euler_derivatives,
               const Matrix3f& moment_of_inertia);

    void step(float time_delta, const Vector3f& thrust, 
                      const Vector3f& drag, const Vector3f &torque);
    //private:
    float _mass = 1.0f;
    Vector3f _position{0.0f, 0.0f, 0.0f};
    Vector3f _velocity{0.0f, 0.0f, 0.0f};
    Vector3f _euler_angles{0.0f, 0.0f, 0.0f};
    Vector3f _euler_derivatives{0.0f, 0.0f, 0.0f};
    //moment of inertia is diagonal due to symmetry
    FrameDrag::Matrix3f _I{1.0f, 0.0f, 0.0f,
                          0.0f, 1.0f, 0.0f,
                          0.0f, 0.0f, 1.0f};
};
}