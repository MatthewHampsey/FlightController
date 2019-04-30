#include "type_conversion.h"

namespace FrameDrag{

Matrix3f quaternionToRotationMatrix(const Quaternion& q)
{
    auto skew = vectorToSkewSymmetricMatrix(q.im());
    auto id = Matrix3f{1.0f, 0.0f, 0.0f,
                    0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 1.0f};
    return id + 2*q.re()*skew + 2*skew*skew;
}

Matrix3f vectorToSkewSymmetricMatrix(const Vector3f& v){
    return Matrix3f{0.0f, -v[2], v[1],
                    v[2], 0.0f, -v[0],
                    -v[1], v[0], 0.0f};
}
}
