#include "../include/pendulum.h"

#include <glm/matrix.hpp>
#include <wrapgl/renderer.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>

#include "../include/math/constants.h"

using namespace glm;

Pendulum::Pendulum(const vec3 &pivot_pos, float rod_length, bool is_static) :
        pivot_(RigidBody(Shape::kCube, 3.0f, vec3(1.0f, 1.0f, 1.0f), is_static)),
        bob_(RigidBody(Shape::kCube, 2.0f, vec3(1.0f, 1.0f, 1.0f), false))
{
        rod_length_ = rod_length;

        pivot_.SetPosition(pivot_pos);
        bob_.SetPosition(vec3(pivot_pos.x, pivot_pos.y - rod_length_, pivot_pos.z));

        // The attachment point will be their center of mass (position)
        pivot_attach_ = vec3(pivot_.GetPosition());
        bob_attach_   = vec3(bob_.GetPosition());

        glGenVertexArrays(1, &line_vao_);
        glGenBuffers(1, &line_vbo_);

        glBindVertexArray(line_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, line_vbo_);

        const GLsizeiptr vertex_size  = sizeof(float) * 6;
        const GLsizeiptr vertex_count = 2;

        // allocate buffer (two vec3 positions, but initially empty)
        glBufferData(GL_ARRAY_BUFFER, vertex_size * vertex_count, nullptr, GL_DYNAMIC_DRAW);

        // describe layout: attribute 0 = vec3 position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, (void*)0);

        // color at location = 1
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, (void*)(sizeof(float) * 3));

        glBindVertexArray(0);
}

void Pendulum::Draw(wgl::Renderer &renderer)
{
        UpdateBobAttachWorld();
        UpdatePivotAttachWorld();

        pivot_.Draw(renderer);
        bob_.Draw(renderer);

        float line[12] = {
        // pivot position + color
        pivot_.GetPosition().x, pivot_.GetPosition().y, pivot_.GetPosition().z,  1.0f, 1.0f, 1.0f,
        // bob position + color
        bob_.GetPosition().x,   bob_.GetPosition().y,   bob_.GetPosition().z,    1.0f, 1.0f, 1.0f
        };

        // update the GPU buffer
        glBindBuffer(GL_ARRAY_BUFFER, line_vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(line), line);

        // bind the shader
        renderer.SetUniformMatrix4f("model", glm::mat4(1.0f));

        // draw the line
        renderer.DrawArrays(line_vao_, GL_LINES, 2);
        glBindVertexArray(0);
}

void Pendulum::UpdatePivotAttachWorld()
{
        pivot_attach_wrld_ = pivot_.GetPosition() + toMat3(pivot_.GetRotation()) * pivot_attach_;
}

void Pendulum::UpdateBobAttachWorld()
{
        bob_attach_wrld_ = bob_.GetPosition() + toMat3(bob_.GetRotation()) * bob_attach_;
}

void Pendulum::Update(float dt)
{
        if (dt <= 0.0f) return;

        UpdatePivotAttachWorld();
        UpdateBobAttachWorld();

        const float kEpsilon = 1e-6f;
        glm::vec3 d = bob_attach_wrld_ - pivot_attach_wrld_;
        float l = glm::length(d);
        if (l < kEpsilon) return;

        glm::vec3 n = d / l;
        glm::vec3 pivot_r = pivot_attach_wrld_ - pivot_.GetPosition();
        glm::vec3 bob_r   = bob_attach_wrld_   - bob_.GetPosition();

        glm::vec3 vel_at_pivot = pivot_.GetLinearVelocity() + glm::cross(pivot_.GetAngularVelocity(), pivot_r);
        glm::vec3 vel_at_bob   = bob_.GetLinearVelocity()   + glm::cross(bob_.GetAngularVelocity(), bob_r);

        float relative_vel_along_rod = glm::dot(vel_at_bob - vel_at_pivot, n);
        float positional_error = l - rod_length_;

        // bias safe
        const float kBiasFactor = 0.2f;
        float bias_vel = (kConstraintStiffness * positional_error) * (kBiasFactor / dt);

        // inverse masses (safe)
        float invMassPivot = pivot_.IsStatic() ? 0.0f : (pivot_.GetMass() > 0.0f ? 1.0f / pivot_.GetMass() : 0.0f);
        float invMassBob   = bob_.GetMass() > 0.0f ? 1.0f / bob_.GetMass() : 0.0f;
        float k_trans = invMassPivot + invMassBob;

        // inverse inertia in world space — if pivot static, use zero matrix
        glm::mat3 invI_pivot = pivot_.IsStatic() ? glm::mat3(0.0f) : pivot_.GetInvInertiaWorld(); // ideally cached
        glm::mat3 invI_bob   = bob_.GetInvInertiaWorld();

        glm::vec3 r_p_cross_n = glm::cross(pivot_r, n);
        glm::vec3 r_b_cross_n = glm::cross(bob_r, n);

        glm::vec3 pivot_rot_resist = invI_pivot * r_p_cross_n;
        glm::vec3 bob_rot_resist   = invI_bob   * r_b_cross_n;

        float k_rot_pivot = glm::dot(r_p_cross_n, pivot_rot_resist);
        float k_rot_bob   = glm::dot(r_b_cross_n, bob_rot_resist);

        float k = k_trans + k_rot_pivot + k_rot_bob;
        if (k < kEpsilon) return;

        float P = - (relative_vel_along_rod + bias_vel) / k;

        // clamp impulse for stability
        const float kMaxImpulse = 1000.0f;
        P = glm::clamp(P, -kMaxImpulse, kMaxImpulse);

        glm::vec3 J = n * P;

        // Apply to bob always
        bob_.IntegrateLinearImpulse(J);
        bob_.IntegrateAngularImpulse(J, bob_r);

        // Do NOT apply to pivot if it's static
        if (!pivot_.IsStatic()) {
                pivot_.IntegrateLinearImpulse(-J);
                pivot_.IntegrateAngularImpulse(-J, pivot_r);
        }

        // Integrate velocities for bodies that can move
        if (!pivot_.IsStatic()) pivot_.IntegrateVelocities(dt);
        bob_.IntegrateVelocities(dt);
}

