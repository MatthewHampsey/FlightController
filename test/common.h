#pragma once

#include <random>
#include <chrono>

#include "vector3f.h"


inline float randomUniformFloat(float max)
{
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    unsigned int seed = value.count();
    std::minstd_rand0 generator (seed);
    std::uniform_real_distribution<float> dist(-100, 100);
    return dist(generator)/100.0f*max;
};

inline FrameDrag::Vector3f randomUniformVector(float max1, float max2, float max3)
{
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
    unsigned int seed = value.count();
    std::minstd_rand0 generator (seed);
    std::uniform_real_distribution<float> dist(-100, 100);
    return FrameDrag::Vector3f{
	    dist(generator)/100.0f*max1,
	    dist(generator)/100.0f*max2,
	    dist(generator)/100.0f*max3
    };
};

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
