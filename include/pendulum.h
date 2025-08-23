#ifndef PENDULUM_H
#define PENDULUM_H

#include <glm/ext/vector_float3.hpp>
#include <wrapgl/renderer.h>

#include "rigid_body.h"

class Pendulum {
        RigidBody pivot_, bob_;

        glm::vec3 pivot_attach_, bob_attach_;

        // World position of the pivot and bob attachment point to each other.
        glm::vec3 pivot_attach_wrld_, bob_attach_wrld_;

        float rod_length_;

        unsigned int line_vao_, line_vbo_; 

public:
        Pendulum(const glm::vec3 &pivot_pos, float rod_length, bool is_static);

        void UpdatePivotAttachWorld();
        void UpdateBobAttachWorld();

        inline RigidBody* GetBob() { return &bob_; }
        inline RigidBody* GetPivot() { return &pivot_; }

        void Update(float dt);

        void Draw(wgl::Renderer &renderer);
};

#endif

