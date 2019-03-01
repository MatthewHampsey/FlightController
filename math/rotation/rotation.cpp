#include "rotation.h"
#include "quaternion.h"
#include "rotation_matrix.h"
#include <math.h>

namespace FrameDrag {

Rotation Rotation::inverse()
{
    return Rotation{ _representation->inverse() };
}

Rotation fromAngleAxis(float angle, const Vector3f& v)
{
    return Rotation{
        Quaternion(cos(angle / 2.0f), sin(angle / 2.0f) * v)
    };
}

Rotation fromAngleAxis(float angle, Vector3f&& v)
{
    return Rotation{
        Quaternion(cos(angle / 2.0f), sin(angle / 2.0f) * v)
    };
}

Rotation fromQuaternion(float q1, float q2, float q3, float q4)
{
    return Rotation{
        Quaternion(q1, q2, q3, q4)
    };
}

Rotation fromMatrix(const std::initializer_list<float>& l)
{
    return Rotation{ RotationMatrix{ l } };
}
}