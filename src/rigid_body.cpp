#include "../include/rigid_body.h"
#include "../include/math/constants.h"
#include "../include/math/math_utils.h"
#include "../include/util/util.h"
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <iostream>
#include <stdexcept>
#include <wrapgl/mesh_factory.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/ext/matrix_transform.hpp>

using namespace glm;

RigidBody::RigidBody(Shape shape, float mass, const vec3 &scale)
{
        mass_ = mass;

        position_ = vec3(0.0f);
        rotation_ = quat(1.0f, 0.0f, 0.0f, 0.0f);
        scale_    = scale;

        linear_velocity_  = vec3(0.0f);
        angular_velocity_ = vec3(0.0f);

        vec3 rgb = vec3(1.0f, 0.2f, 0.3f);

        wgl::VertexLayout layout = GetGlobalVertexLayout();

        switch (shape) {
                case kCube:
                        inertia_tensor_ = GetCubeInertiaTensor(mass_, scale.x, scale.y, scale.z);
                        mesh_ = wgl::MeshFactory::GetCube(layout, &rgb);
                        break;
                case kPyramid:
                        inertia_tensor_ = GetSquarePyramidInertiaTensor(mass_, scale.z, scale.y);
                        mesh_ = wgl::MeshFactory::GetPyramid(layout, &rgb);
                        break;
                default:
                        throw std::runtime_error("What?");
        }

        cached_model_ = mat4(1.0f);
        dirty_ = true;
}

RigidBody::~RigidBody()
{
        wgl::MeshFactory::DestroyMesh(mesh_);
}

void RigidBody::SetPosition(const vec3 &pos)
{
        position_ = pos;
        dirty_ = true;
}

void RigidBody::SetRotation(const quat &rot)
{
        rotation_ = rot;
        dirty_ = true;
}

void RigidBody::SetScale(const vec3 &scale)
{
        scale_ = scale;
        dirty_ = true;
}

void RigidBody::IntegrateLinearAcceleration(const vec3 &force, float dt)
{
        vec3 acc = force / mass_;

        linear_velocity_ += acc * dt;
        linear_velocity_ *= (1 - kDamping * dt);
}

void RigidBody::IntegrateLinearImpulse(const vec3 &impulse)
{
        linear_velocity_ += (impulse / mass_);
}

void RigidBody::IntegrateAngularAcceleration(const vec3 &force, const vec3 &r, float dt)
{
        mat3 R = mat3_cast(rotation_); // convert quaternion to 3x3 rotation
        mat3 I_world = R * inertia_tensor_ * transpose(R);

        // r is just the distance from the center of mass (position_) where
        // the force was applied.
        vec3 applied_torque = cross(r, force);

        // Gyroscopic term.
        vec3 gyro_term = cross(angular_velocity_, (inertia_tensor_ * angular_velocity_));

        // Total torque.
        vec3 net_torque = applied_torque - gyro_term;

        // use I_world to compute angular acceleration
        vec3 angular_acc = inverse(I_world) * net_torque;

        angular_velocity_ += angular_acc * dt;
}

void RigidBody::IntegrateAngularImpulse(const vec3 &impulse, const vec3 &r)
{
        vec3 angular_impulse = cross(r, impulse);

        angular_velocity_ += inverse(inertia_tensor_) * angular_impulse;
}

void RigidBody::IntegrateLinearVelocity(float dt)
{
        position_ += linear_velocity_ * dt; 
        dirty_ = true;
}

void RigidBody::IntegrateAngularVelocity(float dt)
{
        quat omega_quat(0, angular_velocity_.x, angular_velocity_.y, angular_velocity_.z);
        quat q_dot = 0.5f * (omega_quat * rotation_);

        rotation_ += dt * q_dot;
        rotation_ = glm::normalize(rotation_);

        dirty_ = true;
}

void RigidBody::IntegrateAccelerations(const vec3 &forces, const vec3 &r, float dt)
{
        IntegrateLinearAcceleration(forces, dt);
        IntegrateAngularAcceleration(forces, r, dt);
}

void RigidBody::IntegrateImpulses(const vec3 &impulses, const vec3 &r)
{
        IntegrateLinearImpulse(impulses);
        IntegrateAngularImpulse(impulses, r);
}

void RigidBody::IntegrateVelocities(float dt)
{
        IntegrateLinearVelocity(dt);
        IntegrateAngularVelocity(dt);
}

void RigidBody::Draw(wgl::Renderer &renderer)
{
        renderer.SetUniformMatrix4f("model", CalculateModelMatrix());
        mesh_->Draw(renderer);
}

mat4 RigidBody::CalculateModelMatrix()
{
        if (!dirty_) {
                return cached_model_;
        }

        mat4 model = mat4(1.0f);
        model = translate(model, position_);
        model *= toMat4(rotation_);
        model = scale(model, scale_);

        dirty_ = false;

        cached_model_ = model;

        return cached_model_;
}

