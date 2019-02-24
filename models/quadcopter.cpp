#include "quadcopter.h"
#include "euler.h"

namespace FrameDrag {

Quadcopter::Quadcopter(float length, float k_f, float k_t)
    : _prop_controller(length, k_f, k_t) {}

Quadcopter::Quadcopter(float length, float k_f, float k_t, float mass,
                       const Vector3f &position, const Vector3f &velocity,
                       const Vector3f &euler_angles,
                       const Vector3f &euler_derivatives,
                       const Matrix3f &moment_of_inertia)
    : _prop_controller(length, k_f, k_t), _mass(mass), _position{position},
      _velocity{velocity}, _euler_angles{euler_angles},
      _euler_derivatives{euler_derivatives}, _I{moment_of_inertia} {}

void Quadcopter::step(float time_delta, float target_thrust,
                      const Vector3f &drag, const Vector3f &target_torque) {
  Vector3f gravity{0.0f, 0.0f, -_mass * 9.81f};
  auto _bodyframe_to_world = ZYXEulerToRotationMatrix(
      _euler_angles[2], _euler_angles[1], _euler_angles[0]);

  _prop_controller.applyControlTargets(target_thrust, target_torque);
  auto thrust = _prop_controller.getCurrentThrust();
  auto torque = _prop_controller.getCurrentTorque();

  auto linear_acceleration =
      (1.0f / _mass) *
      (gravity + _bodyframe_to_world.apply(Vector3f{0.0f, 0.0f, thrust}) +
       drag);
  auto angular_velocity =
      ZYXEulerToBodyFrameAngularVelocity(_euler_angles, _euler_derivatives);

  auto angular_acceleration =
      _I.inverse() * (torque - angular_velocity.cross(_I * angular_velocity));

  angular_velocity = angular_velocity + time_delta * angular_acceleration;
  _euler_derivatives = BodyFrameAngularVelocityToZYXEulerDerivatives(
      _euler_angles, angular_velocity);
  _euler_angles = _euler_angles + time_delta * _euler_derivatives;
  _velocity = _velocity + time_delta * linear_acceleration;
  _position = _position + time_delta * _velocity;
}

Vector3f Quadcopter::position() { return _position; }

void Quadcopter::setPosition(Vector3f &&p) { _position = std::move(p); }

Vector3f Quadcopter::velocity() { return _velocity; }

void Quadcopter::setVelocity(Vector3f &&v) { _velocity = std::move(v); }

Vector3f Quadcopter::eulerAngles() { return _euler_angles; }

void Quadcopter::setEulerAngles(Vector3f &&e) { _euler_angles = std::move(e); }

Vector3f Quadcopter::eulerAngleDerivatives() { return _euler_derivatives; }

void Quadcopter::setEulerAngleDerivatives(Vector3f &&e) {
  _euler_derivatives = std::move(e);
}

Matrix3f Quadcopter::momentOfInertia() { return _I; }

void Quadcopter::setMomentOfInertia(Matrix3f &&I) { _I = std::move(I); }
}