#ifndef PENDULUM_H
#define PENDULUM_H

#include <glm/ext/vector_float3.hpp>
#include <wrapgl/renderer.h>

#include "rigid_body.h"

class Pendulum {
        RigidBody pivot_, bob_;

        glm::vec3 pivot_attach_, bob_attach_;

        float rod_lenght_;

public:
        Pendulum(const glm::vec3 &pivot_pos, float rod_lenght);

        inline RigidBody* GetBob() { return &bob_; }
        inline RigidBody* GetPivot() { return &pivot_; }

        void Update(float dt);

        void Draw(wgl::Renderer &renderer);
};

#endif

