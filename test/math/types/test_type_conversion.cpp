#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "ConversionTest"
#define BOOST_TEST_MAIN
#include "type_conversion.h"
#include "test_util.h"
#include "common.h"
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/included/unit_test.hpp>

template <typename T> int sgn(T val) {
	    return (T(0) < val) - (val < T(0));
}

BOOST_AUTO_TEST_CASE(test_roundtripQuaternion)
{

  for(int i = 0; i < 500; i++)
  {
    auto axis = randomVector(100.0f);
    axis /= axis.norm();
    float angle = randomUniformFloat(3.159265f);
    FrameDrag::Quaternion q(std::cos(angle/2.0f), std::sin(angle/2.0f)*axis);
    auto euler = FrameDrag::QuaternionToZYXEuler(q);
    auto q_out = FrameDrag::ZYXEulerToQuaternion(euler);
    auto vec = q_out.im();
    float angle_out = 2.0f*acos(q_out.re());
    while(angle_out > 2.0f*3.1459f) angle_out -= 2.0f*3.14159f;
    while(angle_out < -2.0f*3.1459f) angle_out += 2.0f*3.14159f;
    auto axis_out = vec/vec.norm();

    if (sgn(angle) == sgn(angle_out))
    { 
      TEST_CHECK_FLOAT_VALUE(angle_out, angle, 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[0], axis[0], 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[1], axis[1], 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[2], axis[2], 0.0001f);
    }
    else
    {
      TEST_CHECK_FLOAT_VALUE(angle_out, -angle, 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[0], -axis[0], 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[1], -axis[1], 0.0001f);
      TEST_CHECK_FLOAT_VALUE(axis_out[2], -axis[2], 0.0001f);
    }
  }

}

BOOST_AUTO_TEST_CASE(test_roundtripEuler)
{
  for(int i = 0; i < 500; i++)
  {
    auto euler = randomUniformVector(3.14159f, 3.14159f/2.0f, 3.14159f);

    auto q = FrameDrag::ZYXEulerToQuaternion(euler);
    auto euler_out = FrameDrag::QuaternionToZYXEuler(q);
    //std::cout << "euler: " << euler << '\n';
    //std::cout << "roundtripped: " << euler_out << '\n';
    //if(euler_out[0] <= 0.0f) euler_out[0] += 2.0f*3.14159f;
    //if(euler_out[1] <= 0.0f) euler_out[1] += 2.0f*3.14159f;
    //if(euler_out[2] <= 0.0f) euler_out[2] += 2.0f*3.14159f;
    TEST_CHECK_FLOAT_VALUE(euler_out[0], euler[0], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(euler_out[1], euler[1], 0.0001f);
    TEST_CHECK_FLOAT_VALUE(euler_out[2], euler[2], 0.0001f);
  }
}

