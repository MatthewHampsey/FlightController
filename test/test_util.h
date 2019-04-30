#pragma once

#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

namespace Test {
constexpr float EPS = 0.00000001f;
}

#define TEST_CHECK_FLOAT_VALUE(x, y, eps) \
    BOOST_CHECK_SMALL(std::abs((x) - (y)), (eps));
