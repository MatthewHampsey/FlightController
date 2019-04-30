#pragma once

#include <random>

#include "vector3f.h"


inline FrameDrag::Vector3f randomVector(float variance)
{
    std::default_random_engine generator;
    std::normal_distribution<float> distribution(0.0f, variance);
    return FrameDrag::Vector3f{
        distribution(generator),
        distribution(generator),
        distribution(generator)
    };
}