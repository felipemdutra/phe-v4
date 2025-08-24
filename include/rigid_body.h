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

        glm::mat3 cached_inv_inertia_tensor_; // World inverse inertia tensor.
        glm::mat3 cached_inertia_tensor_;     // World inertia tensor.
        glm::mat4 cached_model_;

        wgl::Mesh *mesh_;

        bool is_static_;
        bool dirty_;

public:
        RigidBody() = default;
        RigidBody(Shape shape, float mass, const glm::vec3 &scale, bool is_static);
        ~RigidBody();

        // @brief This method integrates linear acceleration into the linear 
        //        velocity of the body.
        //
        // @param force The force to apply to the body.
        // @param dt Delta time.
        void IntegrateLinearAcceleration(const glm::vec3 &force, float dt);               
        void IntegrateLinearImpulse(const glm::vec3 &impulse);

        void IntegrateAngularAcceleration(const glm::vec3 &force, const glm::vec3 &r, float dt);
        void IntegrateAngularImpulse(const glm::vec3 &impulse, const glm::vec3 &r);

        void IntegrateAccelerations(const glm::vec3 &forces, const glm::vec3 &r, float dt);
        void IntegrateImpulses(const glm::vec3 &impulses, const glm::vec3 &r);

        void IntegrateVelocities(float dt);

        void IntegrateLinearVelocity(float dt);
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

        inline const glm::mat3& GetInertiaTensor() const { return inertia_tensor_; }
        const glm::mat3& GetInvInertiaWorld();
        const glm::mat3& GetInertiaWorld();

        inline const glm::vec3& GetLinearVelocity() const { return linear_velocity_; }
        inline const glm::vec3& GetAngularVelocity() const { return angular_velocity_; }

        inline void SetLinearVelocity(const glm::vec3 new_vel) { linear_velocity_ += new_vel; }
        inline void SetAngularVelocity(const glm::vec3 new_vel) { angular_velocity_ += new_vel; }

        inline float GetMass() const { return mass_; }

        void UpdateCache();

        void Draw(wgl::Renderer &renderer);

        glm::mat4 GetModelMatrix();
};

#endif

