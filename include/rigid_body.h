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
        float mass_;         // Mass of the rigid body (kg).

        glm::vec3 position_; // Position of the center of mass in world space.
        glm::quat rotation_; // Orientation in world space.
        glm::vec3 scale_;    // Local scale.

        glm::vec3 linear_velocity_;
        glm::vec3 angular_velocity_;

        glm::mat3 inertia_tensor_; // Local inertia tensor.

        glm::mat3 cached_inv_inertia_tensor_; // Cached inverse inertia tensor in world space.
        glm::mat3 cached_inertia_tensor_;     // Cached inverse inertia tensor in world space.
        glm::mat4 cached_model_;              // Cached model matrix for rendering.

        wgl::Mesh *mesh_;

        bool is_static_;
        bool dirty_;      // Flag to indicate cache needs updating.

public:
        RigidBody() = default;
        
        // @brief Constructs a rigid body from a shape, mass, scale and static flag.
        //
        // @param shape The shape of the rigid body.
        // @param mass The mass of the body.
        // @param scale Local scale (size).
        // @param is_static If true, the body is immovable.
        RigidBody(Shape shape, float mass, const glm::vec3 &scale, bool is_static);

        // @brief Frees *mesh_.
        ~RigidBody();

        // @brief This method integrates linear acceleration to the linear 
        //        velocity of the body.
        //
        // @param force The force to apply to the body.
        // @param dt Delta time.
        void IntegrateLinearAcceleration(const glm::vec3 &force, float dt);               

        // @brief Integrates a linear impulse to the body's linear velocity.
        //
        // @param impulse Linear impulse to apply.
        void IntegrateLinearImpulse(const glm::vec3 &impulse);

        // @brief Integrates an angular acceleration to the angular velocity
        //        of the body
        // 
        // @param force Torque applied at offset `r`.
        // @param r Position vector from center of mass to application point.
        // @param dt Timestep.
        void IntegrateAngularAcceleration(const glm::vec3 &force, const glm::vec3 &r, float dt);

        // @brief Integrates an angular impulse to the angular velocity
        //        of the body
        // 
        // @param impulse Impulse vector applied at offset `r`.
        // @param r Position vector from center of mass to application point.
        void IntegrateAngularImpulse(const glm::vec3 &impulse, const glm::vec3 &r);

        // @brief Integrates both linear and angular accelerations.
        //
        // @param forces Linear and angular force vector.
        // @param r Position vector from the center of mass for torque 
        //          calculation.
        // @param dt Timestep.
        void IntegrateAccelerations(const glm::vec3 &forces, const glm::vec3 &r, float dt);

        // @brief Integrates both linear and angular impulses.
        //
        // @param impulses Linear and angular impulses.
        // @param r Position vector from center of mass for torque calculation.
        void IntegrateImpulses(const glm::vec3 &impulses, const glm::vec3 &r);

        // @brief Integrates velocities to position and rotation.
        void IntegrateVelocities(float dt);

        // @brief Integrates linear velocity to position
        void IntegrateLinearVelocity(float dt);

        // @brief Integrates angular velocity to rotation
        void IntegrateAngularVelocity(float dt);

        void SetPosition(const glm::vec3 &pos);
        void SetRotation(const glm::quat &rot);
        void SetScale(const glm::vec3 &scale);

        inline void SetStatic() { is_static_ = true; };
        inline void SetStatic(bool value) { is_static_ = value; }

        inline const glm::vec3& GetPosition() const { return position_; }
        inline const glm::quat& GetRotation() const { return rotation_; }
        inline const glm::vec3& GetScale() const { return scale_; }

        inline bool IsStatic() const { return is_static_; }

        // @brief Returns local inertia tensor.
        inline const glm::mat3& GetInertiaTensor() const { return inertia_tensor_; }
        const glm::mat3& GetInvInertiaWorld();
        const glm::mat3& GetInertiaWorld();

        inline const glm::vec3& GetLinearVelocity() const { return linear_velocity_; }
        inline const glm::vec3& GetAngularVelocity() const { return angular_velocity_; }

        inline void SetLinearVelocity(const glm::vec3 new_vel) { linear_velocity_ += new_vel; }
        inline void SetAngularVelocity(const glm::vec3 new_vel) { angular_velocity_ += new_vel; }

        inline float GetMass() const { return mass_; }

        // @brief Updates cached matrices (model, inertia, inverse inertia)
        //        for efficiency.
        void UpdateCache();

        // @brief Draws the rigid body with it's updated cached model matrix.
        //
        // @param renderer The renderer to draw the mesh with.
        void Draw(wgl::Renderer &renderer);

        // @brief Returns the updated cached model matrix.
        glm::mat4 GetModelMatrix();
};

#endif

