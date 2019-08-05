#pragma once

#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

Quaternion grad(const Quaternion& sensor_frame_to_world, const Vector3f& v,
    const Vector3f& measurement);
}