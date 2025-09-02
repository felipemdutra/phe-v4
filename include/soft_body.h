#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <glm/ext/matrix_float3x3.hpp>
#include <vector>

#include "./util/icosphere.h"
#include "./spring.h"
#include "./point.h"

class SoftBody {
        IcoSphere mesh_;

        std::vector<Point> points_;
        std::vector<Spring> springs_;

        double kStiffness;

        bool dirty_;

public:
        // @param position The starting position of the Soft Body.
        // @param point_mass The mass of each point.
        SoftBody(const glm::vec3 &position, float point_mass, double spring_stiffness_factor);

        ~SoftBody();

        void ApplyGravity(double g, float dt);
        
        void UpdateIcoSphereWithPointPositions();

        void Draw(wgl::Renderer &renderer);
};

#endif

