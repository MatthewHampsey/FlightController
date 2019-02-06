#include "rotation.h"
#include "vector3f.h"

namespace FrameDrag{
    Rotation ZYXEulerToRotation(float yaw, float pitch, float roll);
    Vector3f ZYXEulerToAngularVelocity(const Vector3f& euler, 
                                       const Vector3f& euler_derivative);
    Vector3f ZYXEulerToAngularVelocity(float yaw, float pitch, float roll, 
                                       float yaw_deriv, float pitch_deriv, float roll_deriv);
}