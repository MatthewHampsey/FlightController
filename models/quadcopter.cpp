#include "quadcopter.h"
#include "euler.h"

namespace FrameDrag{

Quadcopter::Quadcopter(){}
Quadcopter::Quadcopter(const Vector3f& position,
               const Vector3f& velocity,
               const Vector3f& euler_angles,
               const Vector3f& euler_derivatives,
               const Matrix3f& moment_of_inertia)
        : _position{position}
        , _velocity{velocity}
        , _euler_angles{euler_angles}
        , _euler_derivatives{euler_derivatives}
        , _I{moment_of_inertia} {}

void Quadcopter::step(float time_delta, const Vector3f& thrust, 
                      const Vector3f& drag, const Vector3f &torque){
        Vector3f gravity{0.0f, 0.0f, -_mass*9.81f};
        auto _bodyframe_to_world = ZYXEulerToRotationMatrix(_euler_angles);//.inverse();
        auto linear_acceleration = (1.0f/_mass)*
            (gravity + _bodyframe_to_world.apply(thrust) + drag);
        auto angular_velocity = ZYXEulerToAngularVelocity(_euler_angles,
                                   _euler_derivatives);
        
        auto angular_acceleration 
            = _I.inverse()*(torque - angular_velocity.cross(_I*angular_velocity));

        angular_velocity = angular_velocity + time_delta*angular_acceleration;
        _euler_derivatives = AngularVelocityToZYXEulerDerivatives(_euler_angles, angular_velocity);
        _euler_angles = _euler_angles + time_delta*_euler_derivatives;
        _velocity = _velocity + time_delta*linear_acceleration;
        _position = _position + time_delta*_velocity;
}

}