#ifndef PENDULUM_H
#define PENDULUM_H

#include <glm/ext/vector_float3.hpp>
#include <utility>
#include <vector>
#include <wrapgl/renderer.h>

#include "rigid_body.h"

class Pendulum {
        std::vector<RigidBody> bobs_;

        std::vector<glm::vec3> bobs_attach_;

        // World position of the bobs attachment point to each other.
        // bobs_attach_wrld[i] are the attachment points to bobs_[i]
        std::vector<glm::vec3> bobs_attach_wrld_;

        float rod_length_;

        unsigned int line_vao_, line_vbo_; 

public:
        // @param num_bobs The number of bobs.
        // @param rod_length The length of the rod between each bob.
        // @param is_static Whether the first bob will be static or not.
        Pendulum(const glm::vec3 &pos, size_t num_bobs, float rod_length, bool is_static);

        void UpdateBobAttachWorld(size_t i);

        inline RigidBody* GetBob(size_t i) { return &bobs_[i]; }

        void Update(float dt);

        void Draw(wgl::Renderer &renderer);
};

#endif

