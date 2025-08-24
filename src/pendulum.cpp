#include "../include/pendulum.h"

#include <wrapgl/renderer.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/matrix.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/ext/matrix_transform.hpp>

#include "../include/math/constants.h"

using namespace glm;

Pendulum::Pendulum(const vec3 &pos, size_t num_bobs, float rod_length, bool is_static) :
        rod_length_(rod_length)
{
        bobs_.reserve(num_bobs);
        bobs_attach_.resize(num_bobs);
        bobs_attach_wrld_.resize(num_bobs);

        for (size_t i = 0; i < num_bobs; ++i) {
                glm::vec3 pos_i = (i == 0) ? pos : glm::vec3(
                                bobs_[i-1].GetPosition().x,
                                bobs_[i-1].GetPosition().y - rod_length_,
                                bobs_[i-1].GetPosition().z
                                );

                bobs_.emplace_back(Shape::kCube, 5.0f, glm::vec3(1.0f), i == 0 ? is_static : false);
                bobs_.back().SetPosition(pos_i);
                bobs_attach_.push_back(pos_i);
                bobs_attach_wrld_.push_back(pos_i);
        }
        
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
        if (bobs_.empty()) return;

        // Draw all bodies
        for (size_t i = 0; i < bobs_.size(); ++i) {
                bobs_[i].Draw(renderer);
        }

        // Prepare a buffer for all lines (2 vertices per line, 6 floats per vertex)
        size_t num_lines = bobs_.size() - 1;
        std::vector<float> line_vertices;
        line_vertices.reserve(num_lines * 2 * 6);

        for (size_t i = 1; i < bobs_.size(); ++i) {
                glm::vec3 start = bobs_[i - 1].GetPosition();
                glm::vec3 end   = bobs_[i].GetPosition();

                // Start vertex: position + color
                line_vertices.insert(line_vertices.end(), {start.x, start.y, start.z, 1.0f, 1.0f, 1.0f});
                // End vertex: position + color
                line_vertices.insert(line_vertices.end(), {end.x,   end.y,   end.z,   1.0f, 1.0f, 1.0f});
        }

        // Upload all line vertices at once
        glBindBuffer(GL_ARRAY_BUFFER, line_vbo_);
        glBufferData(GL_ARRAY_BUFFER, line_vertices.size() * sizeof(float), line_vertices.data(), GL_DYNAMIC_DRAW);

        // Bind VAO and shader
        glBindVertexArray(line_vao_);
        renderer.SetUniformMatrix4f("model", glm::mat4(1.0f));

        // Draw all lines in a single draw call
        renderer.DrawArrays(line_vao_, GL_LINES, static_cast<GLsizei>(num_lines * 2));

        glBindVertexArray(0);
}

void Pendulum::UpdateBobAttachWorld(size_t i)
{
        bobs_attach_wrld_[i] = bobs_[i].GetPosition() + toMat3(bobs_[i].GetRotation()) * bobs_attach_[i];
}

void Pendulum::Update(float dt)
{
        if (dt <= 0.0f) return;

        for (size_t i = 1; i < bobs_.size(); i++) {
                UpdateBobAttachWorld(i);
                UpdateBobAttachWorld(i-1);

                RigidBody *pivot = GetBob(i - 1);
                RigidBody *bob   = GetBob(i);

                glm::vec3 bob_attach_wrld = bobs_attach_wrld_[i];
                glm::vec3 pivot_attach_wrld = bobs_attach_wrld_[i-1];

                const float kEpsilon = 1e-6f;
                glm::vec3 d = bob_attach_wrld - pivot_attach_wrld;
                float l = glm::length(d);
                if (l < kEpsilon) return;

                glm::vec3 n = d / l;
                glm::vec3 pivot_r = pivot_attach_wrld - pivot->GetPosition();
                glm::vec3 bob_r   = bob_attach_wrld - bob->GetPosition();

                glm::vec3 vel_at_pivot = pivot->GetLinearVelocity() + glm::cross(pivot->GetAngularVelocity(), pivot_r);
                glm::vec3 vel_at_bob   = bob->GetLinearVelocity()   + glm::cross(bob->GetAngularVelocity(), bob_r);

                float relative_vel_along_rod = glm::dot(vel_at_bob - vel_at_pivot, n);
                float positional_error = l - rod_length_;

                const float kBiasFactor = 0.99f;
                float bias_vel = (kConstraintStiffness * positional_error) * (kBiasFactor / dt);

                // inverse masses (safe)
                float invMassPivot = pivot->IsStatic() ? 0.0f : (pivot->GetMass() > 0.0f ? 1.0f / pivot->GetMass() : 0.0f);
                float invMassBob   = bob->GetMass() > 0.0f ? 1.0f / bob->GetMass() : 0.0f;
                float k_trans = invMassPivot + invMassBob;

                // inverse inertia in world space — if pivot static, use zero matrix
                glm::mat3 invI_pivot = pivot->IsStatic() ? glm::mat3(0.0f) : pivot->GetInvInertiaWorld(); // ideally cached
                glm::mat3 invI_bob   = bob->GetInvInertiaWorld();

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
                bob->IntegrateLinearImpulse(J);
                bob->IntegrateAngularImpulse(J, bob_r);

                // Do NOT apply to pivot if it's static
                if (!pivot->IsStatic()) {
                        pivot->IntegrateLinearImpulse(-J);
                        pivot->IntegrateAngularImpulse(-J, pivot_r);
                }
        }

        for (auto &bob : bobs_) {
                bob.IntegrateVelocities(dt);
        }
}

