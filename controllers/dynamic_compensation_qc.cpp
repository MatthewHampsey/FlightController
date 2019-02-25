#include "dynamic_compensation_qc.h"

namespace FrameDrag {
PDDynamic::PDDynamic(const Matrix3f& moment_of_inertia)
    : _I{ moment_of_inertia }
{
}
Vector3f PDDynamic::getControlVector(const Vector3f& euler_angles,
    const Vector3f& euler_derivatives,
    const Vector3f& target_euler_angles,
    const Vector3f& target_euler_derivatives)
{
    auto dynamic_compensation_approx = Vector3f{
        0.0f, 0.0f, -_I(2, 2) * euler_derivatives[1] * euler_derivatives[0]
    };
    auto euler_error = target_euler_angles - euler_angles;
    auto euler_derivative_error = target_euler_derivatives - euler_derivatives;
    auto control = _K_p * euler_error + _K_d * euler_derivative_error;
    return _I * control + dynamic_compensation_approx;
}

void PDDynamic::setParameters(float damping_factor_x, float natural_frequency_x,
    float damping_factor_y, float natural_frequency_y,
    float damping_factor_z,
    float natural_frequency_z)
{
    _K_p = Matrix3f{ natural_frequency_x * natural_frequency_x, 0.0f, 0.0f,
        0.0f, natural_frequency_y * natural_frequency_y, 0.0f,
        0.0f, 0.0f, natural_frequency_z * natural_frequency_z };

    _K_d = Matrix3f{ 2 * damping_factor_x * natural_frequency_x, 0.0f, 0.0f,
        0.0f, 2 * damping_factor_y * natural_frequency_y, 0.0f,
        0.0f, 0.0f, 2 * damping_factor_z * natural_frequency_z };
}

void PDDynamic::setParameters(float damping_factor, float natural_frequency)
{
    setParameters(damping_factor, natural_frequency, damping_factor,
        natural_frequency, damping_factor, natural_frequency);
}

Matrix3f PDDynamic::K_p() { return _K_p; }

Matrix3f PDDynamic::K_d() { return _K_d; }
}