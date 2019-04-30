#include "matrix.h"
#include "vector3f.h"
#include "vector4f.h"
#include <Eigen/Dense>

namespace FrameDrag {

struct Vector3f::impl {
    Eigen::Vector3f _vec;
};

struct Matrix3f::impl {
    impl(const std::initializer_list<float>& v)
        : _mat{ Eigen::Matrix3f{ v.begin() }.transpose() }
    {
    }
    impl(const impl& other)
        : _mat{ other._mat }
    {
    }
    Eigen::Matrix3f _mat;
};

Matrix3f::Matrix3f()
    : _impl{ new impl{ 0.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 0.0f } }
{
}

Matrix3f::Matrix3f(const std::initializer_list<float>& v)
    : _impl{ new impl{ v } }
{
}

Matrix3f::Matrix3f(const Matrix3f& other)
    : _impl{ new impl{ *other._impl } }
{
}

Matrix3f& Matrix3f::operator=(const Matrix3f& other)
{
    Matrix3f m{ other };
    return *this = std::move(m);
}

void Matrix3f::impl_deleter::operator()(Matrix3f::impl* ptr) const
{
    delete ptr;
}

float& Matrix3f::operator()(size_t i, size_t j) { return _impl->_mat(i, j); }
const float& Matrix3f::operator()(size_t i, size_t j) const { return _impl->_mat(i, j); }


Matrix3f Matrix3f::inverse()
{
    Matrix3f inverse_matrix;
    inverse_matrix._impl->_mat = _impl->_mat.inverse();
    return inverse_matrix;
}

Matrix3f Matrix3f::tranpose()
{
    Matrix3f transpose_matrix;
    transpose_matrix._impl->_mat = _impl->_mat.transpose();
    return transpose_matrix;
}

Matrix3f Matrix3f::operator-() const
{
    Matrix3f mm;
    mm._impl->_mat = -_impl->_mat;
    return mm;
}

Matrix3f Matrix3f::operator-(const Matrix3f& m) const
{
    Matrix3f mm;
    mm._impl->_mat = _impl->_mat - m._impl->_mat;
    return mm;
}

Matrix3f& Matrix3f::operator-=(const Matrix3f& m)
{
    _impl->_mat -= m._impl->_mat;
    return *this;
}

Matrix3f Matrix3f::operator+(const Matrix3f& m) const
{
    Matrix3f mm;
    mm._impl->_mat = _impl->_mat + m._impl->_mat;
    return mm;
}

Matrix3f& Matrix3f::operator+=(const Matrix3f& m)
{
    _impl->_mat += m._impl->_mat;
    return *this;
}

Vector3f operator*(const Matrix3f& m, const Vector3f& v)
{
    Vector3f vv;
    vv._impl->_vec = m._impl->_mat * v._impl->_vec;
    return vv;
}

Matrix3f operator*(const Matrix3f& l, const Matrix3f& r)
{
    Matrix3f mat;
    mat._impl->_mat = l._impl->_mat * r._impl->_mat;
    return mat;
}

Matrix3f operator*(float x, const Matrix3f& m)
{
    Matrix3f m2;
    m2._impl->_mat = x * m._impl->_mat;
    return m2;
}

Matrix3f operator*(const Matrix3f& m, float x) { return x * m; }

Matrix3f operator/(const Matrix3f& m, float x)
{
    Matrix3f m2;
    m2._impl->_mat = m._impl->_mat / x;
    return m2;
}

Vector3f Matrix3f::apply(const Vector3f& v) { return *this * v; }

std::ostream& operator<<(std::ostream& os, const Matrix3f& m)
{
    os << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << '\n'
       << m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << '\n'
       << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << '\n';
    return os;
}

struct Vector4f::impl {
    Eigen::Vector4f _vec;
};

struct Matrix4f::impl {
    impl(const std::initializer_list<float>& v)
        : _mat{ Eigen::Matrix4f{ v.begin() }.transpose() }
    {
    }
    impl(const impl& other)
        : _mat{ other._mat }
    {
    }
    Eigen::Matrix4f _mat;
};

Matrix4f::Matrix4f()
    : _impl{ new impl{ 0.0f, 0.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 0.0f, 0.0f,
          0.0f, 0.0f, 0.0f, 0.0f } }
{
}

Matrix4f::Matrix4f(const std::initializer_list<float>& v)
    : _impl{ new impl{ v } }
{
}

Matrix4f::Matrix4f(const Matrix4f& other)
    : _impl{ new impl{ *other._impl } }
{
}

Matrix4f& Matrix4f::operator=(const Matrix4f& other)
{
    Matrix4f m{ other };
    return *this = std::move(m);
}

void Matrix4f::impl_deleter::operator()(Matrix4f::impl* ptr) const
{
    delete ptr;
}

float& Matrix4f::operator()(size_t i, size_t j) { return _impl->_mat(i, j); }

Matrix4f Matrix4f::inverse()
{
    Matrix4f inverse_matrix;
    inverse_matrix._impl->_mat = _impl->_mat.inverse();
    return inverse_matrix;
}

Matrix4f Matrix4f::transpose()
{
    Matrix4f transpose_matrix;
    transpose_matrix._impl->_mat = _impl->_mat.transpose();
    return transpose_matrix;
}

Matrix4f Matrix4f::operator-() const
{
    Matrix4f mm;
    mm._impl->_mat = -_impl->_mat;
    return mm;
}

Matrix4f Matrix4f::operator-(const Matrix4f& m) const
{
    Matrix4f mm;
    mm._impl->_mat = _impl->_mat - m._impl->_mat;
    return mm;
}

Matrix4f& Matrix4f::operator-=(const Matrix4f& m)
{
    _impl->_mat -= m._impl->_mat;
    return *this;
}

Matrix4f Matrix4f::operator+(const Matrix4f& m) const
{
    Matrix4f mm;
    mm._impl->_mat = _impl->_mat + m._impl->_mat;
    return mm;
}

Matrix4f& Matrix4f::operator+=(const Matrix4f& m)
{
    _impl->_mat += m._impl->_mat;
    return *this;
}

Vector4f operator*(const Matrix4f& m, const Vector4f& v)
{
    Vector4f vv;
    vv._impl->_vec = m._impl->_mat * v._impl->_vec;
    return vv;
}

Matrix4f operator*(const Matrix4f& l, const Matrix4f& r)
{
    Matrix4f mat;
    mat._impl->_mat = l._impl->_mat * r._impl->_mat;
    return mat;
}

Matrix4f operator*(float x, const Matrix4f& m)
{
    Matrix4f m2;
    m2._impl->_mat = x * m._impl->_mat;
    return m2;
}

Matrix4f operator*(const Matrix4f& m, float x) { return x * m; }

Matrix4f operator/(const Matrix4f& m, float x)
{
    Matrix4f m2;
    m2._impl->_mat = m._impl->_mat / x;
    return m2;
}

Vector4f Matrix4f::apply(const Vector4f& v) { return *this * v; }
}