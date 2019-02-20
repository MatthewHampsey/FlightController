#include "dynamic_compensation_qc.h"

namespace FrameDrag{
    PDDynamic::PDDynamic(const Matrix3f &moment_of_inertia)
        : _I{moment_of_inertia} {}
    Vector3f PDDynamic::getControlVector(const Vector3f &euler_angles,
                              const Vector3f &euler_derivatives,
                              const Vector3f &target_euler_angles,
                              const Vector3f &target_euler_derivatives){
        auto dynamic_compensation_approx = 
            Vector3f{0.0f, 0.0f, -_I(2, 2)*euler_derivatives[1]*euler_derivatives[0]};
        auto euler_error = target_euler_angles - euler_angles;
        auto euler_derivative_error = target_euler_derivatives - euler_derivatives;
        auto control = _K_p*euler_error + _K_d*euler_derivative_error;
        return _I*control + dynamic_compensation_approx;
    }
}