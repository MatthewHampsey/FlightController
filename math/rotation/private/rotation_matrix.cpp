#include "rotation_matrix.h"

namespace FrameDrag {
RotationMatrix::RotationMatrix(const std::initializer_list<float> &v)
    : Matrix3f{v} {}

Vector3f RotationMatrix::apply(const Vector3f &v) const { return (*this) * v; }
}