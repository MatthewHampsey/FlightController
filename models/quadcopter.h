#pragma once

#include "matrix.h"
#include "model.h"
#include "prop_controller.h"
#include "rotation.h"
#include "vector3f.h"
#include "vector4f.h"

namespace FrameDrag {
class Quadcopter : Model {
public:
  Quadcopter(float length, float k_f, float k_t);
  Quadcopter(float length, float k_f, float k_t, float mass,
             const Vector3f &position, const Vector3f &velocity,
             const Vector3f &euler_angles, const Vector3f &euler_derivatives,
             const Matrix3f &moment_of_inertia);

  void step(float time_delta, float target_thrust, const Vector3f &drag,
            const Vector3f &target_torque);

  float mass();
  void setMass(float mass);
  Vector3f position();
  void setPosition(Vector3f &&p);
  Vector3f velocity();
  void setVelocity(Vector3f &&v);
  Vector3f eulerAngles();
  void setEulerAngles(Vector3f &&e);
  Vector3f eulerAngleDerivatives();
  void setEulerAngleDerivatives(Vector3f &&e);
  Matrix3f momentOfInertia();
  void setMomentOfInertia(Matrix3f &&I);

private:
  PropellerController _prop_controller;
  float _mass = 1.0f;
  Vector3f _position{0.0f, 0.0f, 0.0f};
  Vector3f _velocity{0.0f, 0.0f, 0.0f};
  Vector3f _euler_angles{0.0f, 0.0f, 0.0f};
  Vector3f _euler_derivatives{0.0f, 0.0f, 0.0f};
  Matrix3f _I{1.0f, 0.0f, 0.0f,
              0.0f, 1.0f, 0.0f,
              0.0f, 0.0f, 2.0f};
};
}