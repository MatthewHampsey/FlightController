#include "dynamic_compensation_qc.h"
#include "euler.h"
#include "quadcopter.h"

#include <iostream>
#include <random>

using namespace FrameDrag;
int main() {
  Quadcopter q{1.0f, 1.0f, 1.0f};
  q.setPosition(Vector3f{0.0f, 0.0f, 30.0f});
  q.setEulerAngles(Vector3f{0.1f, 0.0f, 0.0f});
  PDDynamic controller{q.momentOfInertia()};
  controller._K_p =
      Matrix3f{0.1f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.1f};
  controller._K_d =
      Matrix3f{0.03f, 0.0f, 0.0f, 0.0f, 0.03f, 0.0f, 0.0f, 0.0f, 0.03f};
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, 1.0};

  for (int i = 0; i < 100000; i++) {

    // std::cout << d(gen) << '\n';
    float time_delta = 0.1;
    float thrust = 9.81f;
    Vector3f drag{(float)d(gen), (float)d(gen), (float)d(gen)};
    Vector3f target_euler;
    if (i >= 20000) {
      target_euler = Vector3f(0.0f, 0.0f, 0.9f);
    } else {
      target_euler = Vector3f(0.0f, 0.1f, 0.4f);
    }
    Vector3f target_euler_derivatives(0.0f, 0.0f, 0.0f);
    Vector3f torque =
        controller.getControlVector(q.eulerAngles(), q.eulerAngleDerivatives(),
                                    target_euler, target_euler_derivatives);
    q.step(time_delta, thrust, drag, torque);
    std::cout << "angles: " << q.eulerAngles() << '\n';
    // std::cout << "prop speeds: " << q._prop_speed << '\n';
    // std::cout << q._position << '\n';
    // std::cout << q._velocity << '\n';
  }
  // controller._K_p = Matrix3f{};

  return 0;
}