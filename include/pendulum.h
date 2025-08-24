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

        // @brief Updates the world-space attachment point for a specific bob.
        //
        // This computes the global position of the attachment point of the
        // i-th bob by transforming its local attachment point with the bob's
        // current rotation and position.
        //
        // @param i The index of the bob to update.
        void UpdateBobAttachWorld(size_t i);

        // @brief Returns a pointer to a specific bob.
        //
        // @returns A pointer to the i-th bob.
        inline RigidBody* GetBob(size_t i) { return &bobs_[i]; }

        // @brief Updates the pendulum system for a given timestep
        //
        // This method enforces the rigid rod constraints between consecutive
        // bobs in the pendulum chain and integrates their velocities.
        // For each bob:
        //     - The world-space attachment points are updated.
        //     - The relative velocity along the rod and positional error are
        //           computed.
        //     - A corrective bias velocity is calculated to enforce the rod
        //           length.
        //     - Linear and angular impulses are applied to each bob to satisfy
        //           constraint.
        //     - Static bobs are not modified.
        // 
        // After constraint resolution, all bobs integrate their linear and
        // angular velocities over the timestep.
        //
        // @param dt The simulation timestep. Must be positive.
        void Update(float dt);

        // @brief Renders the pendulum and its connecting rods.
        //
        // This function draws all the pendulum bobs and the rods connecting
        // them. Each bob is drawn individually, and then line segments are
        // created between them to represent the rods.
        //
        // @param renderer The renderer used for drawing.
        void Draw(wgl::Renderer &renderer);
};

#endif

