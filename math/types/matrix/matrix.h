#pragma once
#include "matrix.h"
#include "vector.h"
#include <initializer_list>
#include <memory>
#include <Eigen/Dense>

namespace FrameDrag {

template <int N, int M>
class Matrix{
	public:
Matrix()
{
  _mat = Eigen::Matrix<float, N, M>::Zero();
}

Matrix(const std::initializer_list<float>& v)
    : _mat{ Eigen::Matrix<float, N, M>{ v.begin() }.transpose() }
{
}

Matrix(const Matrix& other)
    : _mat{ other._mat}
{
}

Matrix& operator=(const Matrix& other)
{
    _mat = other._mat;
    return *this;
}

float& operator()(size_t i, size_t j) { return _mat(i, j); }
const float& operator()(size_t i, size_t j) const { return _mat(i, j); }


Matrix inverse()
{
    Matrix inverse_matrix;
    inverse_matrix._mat = _mat.inverse();
    return inverse_matrix;
}

Matrix tranpose()
{
    Matrix transpose_matrix;
    transpose_matrix._mat = _mat.transpose();
    return transpose_matrix;
}

Matrix operator-() const
{
    Matrix mm;
    mm._mat = -_mat;
    return mm;
}

Matrix operator-(const Matrix& m) const
{
    Matrix mm;
    mm._mat = _mat - m._mat;
    return mm;
}

Matrix& operator-=(const Matrix& m)
{
    _mat -= m._mat;
    return *this;
}

Matrix operator+(const Matrix& m) const
{
    Matrix mm;
    mm._mat = _mat + m._mat;
    return mm;
}

Matrix& operator+=(const Matrix& m)
{
    _mat += m._mat;
    return *this;
}
    template <int O>
    friend Vector<O> operator*(const Matrix<O, O>& m, const Vector<O>& v);
    template <int O, int P, int Q>
    friend Matrix<O, Q> operator*(const Matrix<O, P>& l, const Matrix<P, Q>& r);
    template <int O, int P>
    friend Matrix<O, P> operator*(float x, const Matrix<O, P>& m);
    template <int O, int P>
    friend Matrix<O, P> operator*(const Matrix<O, P>& m, float x);
    template <int O, int P>
    friend Matrix<O, P> operator/(const Matrix<O, P>& m, float x);
    template <int O, int P>
    friend std::ostream& operator<<(std::ostream& os, const Matrix<O, P>& v);

Vector<N> apply(const Vector<N>& v) { return *this * v; }

private:

    Eigen::Matrix<float, N, M> _mat;
};

template <int N>
Vector<N> operator*(const Matrix<N, N>& m, const Vector<N>& v)
{
    Vector<N> vv;
    vv._vec = m._mat * v._vec;
    return vv;
}


template <int N, int M, int O>
Matrix<N, O> operator*(const Matrix<N, M>& l, const Matrix<M, O>& r)
{
    Matrix<N, O> mat;
    mat._mat = l._mat * r._mat;
    return mat;
}

template <int N, int M>
Matrix<N, M> operator*(float x, const Matrix<N, M>& m)
{
    Matrix<N, M> m2;
    m2._mat = x * m._mat;
    return m2;
}

template <int N, int M>
Matrix<N, M> operator*(const Matrix<N, M>& m, float x) { return x * m; }

template<int N, int M>
Matrix<N, M> operator/(const Matrix<N, M>& m, float x)
{
    Matrix<N, M> m2;
    m2._mat = m._mat / x;
    return m2;
}


template<int N, int M>
std::ostream& operator<<(std::ostream& os, const Matrix<N, M>& m)
{
    for(int i = 0; i < M; i++)
    {
      for(int j = 0; j < N-1; j++)
      {
       os << m(i, j) << " ";
      }
      if (N > 0) os << m(i, N-1);
      os  << '\n';
    }
    return os;
}

using Matrix3f = Matrix<3, 3>;
using Matrix4f = Matrix<4, 4>;

}
