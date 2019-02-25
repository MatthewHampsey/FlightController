#pragma once
#include "vector3f.h"
#include <initializer_list>
#include <iostream>
#include <memory>

namespace FrameDrag {

class Rotation {
private:
    struct RotationPrivateBase {
        virtual ~RotationPrivateBase() = default;
        virtual Vector3f apply(const Vector3f& other) = 0;
        virtual std::unique_ptr<RotationPrivateBase> inverse() = 0;
    };

    template <typename T>
    struct RotationPrivate : RotationPrivateBase {
        T m_t;
        RotationPrivate(T&& t)
            : m_t(std::move(t))
        {
        }
        Vector3f apply(const Vector3f& other) override { return m_t.apply(other); }
        std::unique_ptr<RotationPrivateBase> inverse() override { return std::make_unique<RotationPrivate<T> >(m_t.inverse()); }
    };

    Rotation(std::unique_ptr<RotationPrivateBase>&& t)
    {
        _representation = std::move(t);
    }

public:
    template <typename T>
    Rotation(T&& t)
    {
        _representation = std::make_unique<RotationPrivate<T> >(std::move(t));
    }

    template <typename T>
    T apply(const T& v) { return _representation->apply(v); }

    Rotation inverse();

private:
    std::unique_ptr<RotationPrivateBase> _representation;
};

Rotation fromAngleAxis(float angle, const Vector3f& v);
Rotation fromAngleAxis(float angle, Vector3f&& v);
Rotation fromQuaternion(float q1, float q2, float q3, float q4);
Rotation fromMatrix(const std::initializer_list<float>& l);
}