#include "euler.h"
#include "rotation.h"
#include "vector3f.h"
#include <iostream>
#include <vector>

int main() {

  // rotate by pi around x-axis

  std::vector<FrameDrag::Rotation> rots;
  // interface is completely decoupled from representation
  rots.push_back(FrameDrag::fromAngleAxis(3.14159265f,
                                          FrameDrag::Vector3f{1.0, 0.0, 0.0}));
  rots.push_back(FrameDrag::fromMatrix(
      {1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f}));
  rots.push_back(FrameDrag::ZYXEulerToRotation(3.14159265, 0.0f, 0.0f));
  auto v = FrameDrag::Vector3f{0.0f, 1.0f, 0.0f};
  std::cout << v << '\n';
  std::cout << rots[0].apply(v) << '\n';
  std::cout << rots[1].apply(v) << '\n';
  std::cout << rots[2].apply(v) << '\n';
  // Eigen::Vector3f v{};
  return 0;
}
