#include "conversion.h"

namespace FrameDrag {

Quaternion toQuaternion(const Vector4f& vec)
{
    return Quaternion{ vec[0], vec[1], vec[2], vec[3] };
}
}