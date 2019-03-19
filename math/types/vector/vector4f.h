#pragma once

#include <initializer_list>
#include <memory>

namespace FrameDrag {

class Matrix4f;

// Can't template this out because of pimpl. This should be
// refactored if/when the pimpl is replaced.
class Vector4f {

    struct impl;
    struct impl_deleter {
        void operator()(impl*) const;
    };

public:
    Vector4f();
    Vector4f(float w, float x, float y, float z);
    Vector4f(const std::initializer_list<float>& l);
    Vector4f(const Vector4f& other);
    Vector4f& operator=(const Vector4f& other);
    Vector4f(Vector4f&&) = default;
    Vector4f& operator=(Vector4f&&) = default;

    float& operator[](size_t i);
    const float& operator[](size_t i) const;

    bool operator==(const Vector4f& v) const;

    Vector4f operator-() const;
    Vector4f operator-(const Vector4f& v) const;
    Vector4f& operator-=(const Vector4f& v);
    Vector4f operator+(const Vector4f& v) const;
    Vector4f& operator+=(const Vector4f& v);
    Vector4f& operator*=(float f);
    Vector4f& operator/=(float f);
    float innerProduct(const Vector4f& v) const;
    Vector4f reverse() const;

    friend Vector4f operator*(float x, const Vector4f& v);
    friend Vector4f operator*(const Vector4f& v, float x);
    friend Vector4f operator*(const Matrix4f& m, const Vector4f& v);
    friend Vector4f operator/(const Vector4f& v, float x);
    friend std::ostream& operator<<(std::ostream& os, const Vector4f& v);

private:
    std::unique_ptr<impl, impl_deleter> _impl;
};
}