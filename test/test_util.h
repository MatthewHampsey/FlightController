#pragma once

#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

namespace Test{
    constexpr float EPS = 0.00000001f;
}

#define TEST_CHECK_FLOAT_VALUE(x, y, eps) BOOST_CHECK_SMALL(std::abs((x) - (y)), (eps));
