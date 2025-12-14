#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <array>
#include <glm/ext/matrix_float3x3.hpp>
#include <unordered_map>
#include <vector>

#include "./util/icosphere.h"
#include "./spring.h"
#include "./point.h"

struct State {
        glm::vec3 x; // Position.
        glm::vec3 v; // Velocity.
};

struct StateDerivative {
        glm::vec3 v; // Velocity is the derivative of position.
        glm::vec3 a; // Acceleration is the derivative of velocity.
};

class SoftBody {
        static constexpr float kFloorY = 0.0f;

        IcoSphere mesh_;

        std::vector<Point> points_;
        std::vector<Spring> springs_;

        float rest_volume_;

        const float kElasticConstant;
        bool dirty_;

        void SetupSprings();

        float CalculateVolume(const std::vector<State> &states);
        float CalculateVolume();

        // @brief Computes the derivative state of each body based on given
        //        states. states[i] = points_[i].
        std::vector<StateDerivative> ComputeStatesDerivative(const std::vector<State> &states);

        std::vector<State> BuildStates();

        glm::vec3 GetCenterPos(const std::vector<State> &states);

public:
        // @param position The starting position of the Soft Body.
        // @param point_mass The mass of each point.
        SoftBody(const glm::vec3 &position, float point_mass, float kElasticConstant);

        ~SoftBody();

        void Update(float dt);

        // Temporary method while I don't implement actual colisions.
        void ResolveFloorCollision(float dt);
        
        void UpdateIcoSphereWithPointPositions();

        void Draw(wgl::Renderer &renderer);
};

#endif

