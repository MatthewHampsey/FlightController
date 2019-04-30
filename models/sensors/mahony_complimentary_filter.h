#pragma once

#include <vector>
#include "grad.h"
#include "quaternion.h"
#include "vector3f.h"

namespace FrameDrag {

class MahonyFilter {

public:
    MahonyFilter();
    MahonyFilter(const Quaternion& q,
        std::vector<std::pair<unsigned int, Vector3f>> reference_directions,
        float k_p, float k_i);
    MahonyFilter(Quaternion&& q,
        std::vector<std::pair<unsigned int, Vector3f>> reference_directions,
        float k_p, float k_i);

    void update(const Vector3f& gyro_m,
        std::vector<Vector3f> measurements,
        float time_delta);

    Quaternion& estimate();

private:
    Quaternion _current_estimate;
    Vector3f _estimated_bias;
    std::vector<std::pair<unsigned int, Vector3f>> _reference_directions;
    float _k_p;
    float _k_i;
};
}