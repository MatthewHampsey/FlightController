#include "vector4f.h"
#include <Eigen/Dense>

namespace FrameDrag {

struct Vector4f::impl {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    impl(float w, float x, float y, float z)
        : _vec{ Eigen::Vector4f{ w, x, y, z } }
    {
    }
    impl(const std::initializer_list<float>& l)
        : _vec{ l.begin() }
    {
    }
    impl(const impl& other)
        : _vec{ other._vec }
    {
    }
    inline float& operator[](size_t i) { return _vec[i]; }
    inline const float& operator[](size_t i) const { return _vec[i]; }
    Eigen::Vector4f _vec;
};

Vector4f::Vector4f()
    : _impl{ new impl{ 0.0f, 0.0f, 0.0f, 0.0f } }
{
}
Vector4f::Vector4f(float w, float x, float y, float z)
    : _impl{ new impl{ w, x, y, z } }
{
}

Vector4f::Vector4f(const std::initializer_list<float>& l)
    : _impl{ new impl{ l } }
{
}

Vector4f::Vector4f(const Vector4f& other)
    : _impl{ new impl{ *other._impl } }
{
}

Vector4f& Vector4f::operator=(const Vector4f& other)
{
    Vector4f v{ other };
    return *this = std::move(v);
}

void Vector4f::impl_deleter::operator()(Vector4f::impl* ptr) const
{
    delete ptr;
}

float& Vector4f::operator[](size_t i) { return _impl->operator[](i); }
const float& Vector4f::operator[](size_t i) const
{
    return _impl->operator[](i);
}

bool Vector4f::operator==(const Vector4f& v) const
{
    return (_impl->_vec == v._impl->_vec);
}

Vector4f Vector4f::operator-() const
{
    Vector4f vv;
    vv._impl->_vec = -_impl->_vec;
    return vv;
}

Vector4f Vector4f::operator-(const Vector4f& v) const
{
    Vector4f vv;
    vv._impl->_vec = _impl->_vec - v._impl->_vec;
    return vv;
}

Vector4f& Vector4f::operator-=(const Vector4f& v)
{
    _impl->_vec -= v._impl->_vec;
    return *this;
}

Vector4f Vector4f::operator+(const Vector4f& v) const
{
    Vector4f vv;
    vv._impl->_vec = _impl->_vec + v._impl->_vec;
    return vv;
}

Vector4f& Vector4f::operator+=(const Vector4f& v)
{
    _impl->_vec += v._impl->_vec;
    return *this;
}

Vector4f Vector4f::reverse() const
{
    Vector4f vv;
    vv._impl->_vec = _impl->_vec.reverse();
    return vv;
}

Vector4f operator*(float x, const Vector4f& v)
{
    Vector4f v2;
    v2._impl->_vec = x * (v._impl->_vec);
    return v2;
}

Vector4f operator*(const Vector4f& v, float x) { return x * v; }

Vector4f operator/(const Vector4f& v, float x)
{
    Vector4f v2;
    v2._impl->_vec = v._impl->_vec / x;
    return v2;
}

std::ostream& operator<<(std::ostream& os, const Vector4f& v)
{
    os << v[0] << " " << v[1] << " " << v[2] << " " << v[3];
    return os;
}
}