#pragma once
#include "matrix.h"
#include <initializer_list>
#include <memory>

namespace FrameDrag {

class Vector3f;
class Vector4f;

class Matrix3f {
    struct impl;
    struct impl_deleter {
        void operator()(impl*) const;
    };

public:
    Matrix3f();
    Matrix3f(const std::initializer_list<float>& v);
    Matrix3f(const Matrix3f& other);
    Matrix3f& operator=(const Matrix3f& other);
    Matrix3f(Matrix3f&&) = default;
    Matrix3f& operator=(Matrix3f&&) = default;
    Matrix3f inverse();

    Matrix3f operator-() const;
    Matrix3f operator-(const Matrix3f& other) const;
    Matrix3f& operator-=(const Matrix3f& other);
    Matrix3f operator+(const Matrix3f& other) const;
    Matrix3f& operator+=(const Matrix3f& other);
    bool isApprox(const Matrix3f& other, float prec) const;
    float& operator()(size_t i, size_t j);
    friend Vector3f operator*(const Matrix3f& m, const Vector3f& v);
    friend Matrix3f operator*(const Matrix3f& l, const Matrix3f& r);
    friend Matrix3f operator*(float x, const Matrix3f& m);
    friend Matrix3f operator*(const Matrix3f& m, float x);
    friend Matrix3f operator/(const Matrix3f& m, float x);

    Vector3f apply(const Vector3f& v);

private:
    std::unique_ptr<impl, impl_deleter> _impl;
};

// Can't template this out because of pimpl. This should be
// refactored if/when the pimpl is replaced.
class Matrix4f {
    struct impl;
    struct impl_deleter {
        void operator()(impl*) const;
    };

public:
    Matrix4f();
    Matrix4f(const std::initializer_list<float>& v);
    Matrix4f(const Matrix4f& other);
    Matrix4f& operator=(const Matrix4f& other);
    Matrix4f(Matrix4f&&) = default;
    Matrix4f& operator=(Matrix4f&&) = default;
    Matrix4f inverse();

    Matrix4f operator-() const;
    Matrix4f operator-(const Matrix4f& other) const;
    Matrix4f& operator-=(const Matrix4f& other);
    Matrix4f operator+(const Matrix4f& other) const;
    Matrix4f& operator+=(const Matrix4f& other);
    bool isApprox(const Matrix4f& other, float prec) const;
    float& operator()(size_t i, size_t j);
    friend Vector4f operator*(const Matrix4f& m, const Vector4f& v);
    friend Matrix4f operator*(const Matrix4f& l, const Matrix4f& r);
    friend Matrix4f operator*(float x, const Matrix4f& m);
    friend Matrix4f operator*(const Matrix4f& m, float x);
    friend Matrix4f operator/(const Matrix4f& m, float x);

    Vector4f apply(const Vector4f& v);

private:
    std::unique_ptr<impl, impl_deleter> _impl;
};
}