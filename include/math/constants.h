#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <glm/glm.hpp>

constexpr float kConstraintStiffness = 0.99f;
constexpr float kGravity        = -9.8f;
constexpr float kElasticConstant = 0.1f;  // now very soft
constexpr float kElasticDamping  = 0.2f;  // damping along springs
constexpr float kGlobalDamping   = 0.05f; // damping for all points
constexpr float kFloorY          = 0.0f;
constexpr float kFloorStiffness  = kElasticConstant * 3.0f; // 0.3
constexpr float kFloorDamping    = kElasticDamping * 2.0f; // 0.4
constexpr float kPressure        = 100.0f; // reduced
#endif
