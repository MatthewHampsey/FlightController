#include "derivative.h"
#include "euler.h"
#include "quadcopter.h"
#include "quat_control.h"
#include "type_conversion.h"
#include <iostream>
#include <random>

using namespace FrameDrag;

int main()
{
    Quadcopter q{ 1.0f, 1.0f, 1.0f };
    q.setPosition(Vector3f{ 0.0f, 0.0f, 30.0f });
    q.setEulerAngles(Vector3f{ 0.1f, 0.0f, 0.0f });
    QuaternionController controller{ q.momentOfInertia() };
    controller.setParameters(1.0f, 1.0f);
    std::random_device rd{};
    std::mt19937 gen{ rd() };
    std::normal_distribution<> d{ 0, 1.0 };
    Quaternion prev_quat;
    Quaternion prev_target_quat;
    for (int i = 0; i < 100000; i++) {
        // std::cout << d(gen) << '\n';
        float time_delta = 0.1;
        float thrust = 9.81f;
        Vector3f drag{ (float)d(gen), (float)d(gen), (float)d(gen) };
        Vector3f target_euler;
        if (i >= 20000) {
            target_euler = Vector3f{0.0f, 0.0f, 0.9f};
        } else {
            target_euler = Vector3f{0.0f, 0.1f, 0.4f};
        }
        auto current_quat = ZYXEulerToQuaternion(q.eulerAngles());
        auto quat_deriv = numerical_derivative(prev_quat, current_quat, time_delta);
        auto current_target_quat = ZYXEulerToQuaternion(target_euler);
        auto target_quat_deriv = numerical_derivative(prev_target_quat, current_target_quat, time_delta);
        Vector3f torque = controller.getControlVector(
            current_quat, quat_deriv, current_target_quat, target_quat_deriv);
        std::cout << "target angles: " << target_euler << '\n';
        std::cout << "current angles: " << q.eulerAngles() << '\n';
        q.step(time_delta, thrust, drag, torque);
        prev_quat = current_quat;
        prev_target_quat = current_target_quat;
    }

    return 0;
}
