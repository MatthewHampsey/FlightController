#pragma once
#include <Eigen/Dense>

namespace FrameDrag {

template<int N, int M>
struct Matrix;

template <int N>
class Vector{

public:
Vector()
{
  _vec = Eigen::Matrix<float, N, 1>::Zero();
}

Vector(const std::initializer_list<float>& l)
    : _vec{ l.begin() }
{
}

Vector(const Vector& other)
    : _vec{ other._vec }
{
}

Vector& operator=(const Vector& other)
{
    _vec = other._vec;
    return *this;
}

float& operator[](size_t i) { return _vec[i]; }
const float& operator[](size_t i) const
{
    return _vec[i];
}

bool operator==(const Vector& v) const
{
    return (_vec == v._vec);
}

Vector operator-() const
{
    Vector vv;
    vv._vec = -_vec;
    return vv;
}

Vector operator-(const Vector& v) const
{
    Vector vv;
    vv._vec = _vec - v._vec;
    return vv;
}

Vector& operator-=(const Vector& v)
{
    _vec -= v._vec;
    return *this;
}

Vector operator+(const Vector& v) const
{
    Vector vv;
    vv._vec = _vec + v._vec;
    return vv;
}

Vector& operator+=(const Vector& v)
{
    _vec += v._vec;
    return *this;
}

Vector& operator*=(float f)
{
    _vec *= f;
    return *this;
}

Vector& operator/=(float f)
{
    _vec /= f;
    return *this;
}

Vector cross(const Vector& v) const
{
    Vector vv;
    vv._vec = _vec.cross(v._vec);
    return vv;
}

float innerProduct(const Vector& v) const
{
    return _vec.dot(v._vec);
}

Vector reverse() const
{
    Vector vv;
    vv._vec = _vec.reverse();
    return vv;
}

float norm() const
{
    return _vec.norm();
}

    template <int M>
    friend Vector<M> operator*(float x, const Vector<M>& v);
    template <int M>
    friend Vector<M> operator*(const Vector<M>& v, float x);
    template <int M, int O>
    friend Vector<M> operator*(const Matrix<M, O>& m, const Vector<O>& v);
    template <int M>
    friend Vector<M> operator/(const Vector<M>& v, float x);
    template <int M>
    friend std::ostream& operator<<(std::ostream& os, const Vector<M>& v);

private:
    Eigen::Matrix<float, N, 1> _vec;
};

template <int N>
Vector<N> operator*(float x, const Vector<N>& v)
{
    Vector<N> v2;
    v2._vec = x * (v._vec);
    return v2;
}

template <int N>
Vector<N> operator*(const Vector<N>& v, float x) { return x * v; }

template <int N>
Vector<N> operator/(const Vector<N>& v, float x)
{
    Vector<N> v2;
    v2._vec = v._vec / x;
    return v2;
}

template <int N>
std::ostream& operator<<(std::ostream& os, const Vector<N>& v)
{
    for (int i = 0; i < N - 1; i++)
    {
      os << v[i] << " ";
    }
    if(N > 0) os << v[N-1];
    return os;
}

}
