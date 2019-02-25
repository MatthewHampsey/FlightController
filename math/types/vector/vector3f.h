#pragma once

#include <initializer_list>
#include <memory>

namespace FrameDrag {

class Matrix3f;

class Vector3f {

    struct impl;
    struct impl_deleter {
        void operator()(impl*) const;
    };

public:
    Vector3f();
    Vector3f(float x, float y, float z);
    Vector3f(const std::initializer_list<float>& l);
    Vector3f(const Vector3f& other);
    Vector3f& operator=(const Vector3f& other);
    Vector3f(Vector3f&&) = default;
    Vector3f& operator=(Vector3f&&) = default;

    float& operator[](size_t i);
    const float& operator[](size_t i) const;

    bool operator==(const Vector3f& v) const;

    Vector3f operator-() const;
    Vector3f operator-(const Vector3f& v) const;
    Vector3f& operator-=(const Vector3f& v);
    Vector3f operator+(const Vector3f& v) const;
    Vector3f& operator+=(const Vector3f& v);
    Vector3f cross(const Vector3f& v) const;
    Vector3f reverse() const;

    friend Vector3f operator*(float x, const Vector3f& v);
    friend Vector3f operator*(const Vector3f& v, float x);
    friend Vector3f operator*(const Matrix3f& m, const Vector3f& v);
    friend Vector3f operator/(const Vector3f& v, float x);
    friend std::ostream& operator<<(std::ostream& os, const Vector3f& v);

private:
    std::unique_ptr<impl, impl_deleter> _impl;
};
}