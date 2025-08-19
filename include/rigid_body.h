#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/vector_float3.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <wrapgl/mesh.h>
#include <wrapgl/renderer.h>

enum Shape {
        kCube,
        kPyramid,
};

class RigidBody {
        float mass_;

        glm::vec3 position_; // position_ is also the position of center of mass. 
        glm::quat rotation_;
        glm::vec3 scale_;

        glm::vec3 linear_velocity_;
        glm::vec3 angular_velocity_;

        glm::mat3 inertia_tensor_;

        glm::mat4 cached_model_;
        wgl::Mesh *mesh_;

        bool dirty_;

public:
        RigidBody(Shape shape, float mass, const glm::vec3 &scale);
        ~RigidBody();

        // @brief This method integrates linear acceleration into the linear 
        //        velocity of the body.
        //
        // @param force The force to apply to the body.
        // @param dt Delta time.
        void IntegrateLinearAcceleration(glm::vec3 force, float dt);               
        void IntegrateLinearImpulse(glm::vec3 impulse);

        void IntegrateAngularAcceleration(glm::vec3 force, glm::vec3 r, float dt);
        void IntegrateAngularImpulse(glm::vec3 impulse, glm::vec3 r);

        void IntegrateAccelerations(glm::vec3 forces, glm::vec3 r, float dt);
        void IntegrateImpulses(glm::vec3 impulses, glm::vec3 r, float dt);

        void IntegrateVelocities(float dt);

        void IntegrateLinearVelocity(float dt);
        void IntegrateAngularVelocity(float dt);

        void Draw(wgl::Renderer &renderer);

        glm::mat4 CalculateModelMatrix();
};

#endif

