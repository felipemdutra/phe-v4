#include "../include/pendulum.h"

#include <algorithm>
#include <cmath>
#include <glm/matrix.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/glm.hpp>

#include "../include/math/constants.h"

using namespace glm;

Pendulum::Pendulum(const vec3 &pivot_pos, float rod_lenght) :
        pivot_(RigidBody(Shape::kCube, 3.0f, vec3(1.0f, 1.0f, 1.0f))),
        bob_(RigidBody(Shape::kCube, 2.0f, vec3(1.0f, 1.0f, 1.0f)))
{
        rod_lenght_ = rod_lenght;

        pivot_.SetPosition(pivot_pos);
        bob_.SetPosition(vec3(pivot_pos.x, pivot_pos.y - rod_lenght_, pivot_pos.z));

        // The attachment point will be their center of mass (position)
        pivot_attach_ = vec3(pivot_.GetPosition().x, pivot_.GetPosition().y + pivot_.GetScale().y, pivot_.GetPosition().z);
        bob_attach_   = vec3(bob_.GetPosition());
}

void Pendulum::Update(float dt)
{
        // World position of the pivot attachment point to the bob.
        vec3 pivot_attach_wrld = pivot_.GetPosition() + toMat3(pivot_.GetRotation()) * pivot_attach_;

        // World position of the bob attachment point to the pivot.
        vec3 bob_attach_wrld = bob_.GetPosition() + toMat3(bob_.GetRotation()) * bob_attach_;
        
        // d is just the straight line vector from the pivot to the bob in
        // world space.
        vec3 d = bob_attach_wrld - pivot_attach_wrld;
        const float eps = 1e-6f;

        // l is the lenght of d. AKA the real distance between the pivot and
        // the bob in the current frame.
        float l = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

        if (l < eps) {
                return;
        }

        // n is the normalized direction along the rod.
        vec3 n = d / l;

        vec3 pivot_r = pivot_attach_wrld - pivot_.GetPosition();
        vec3 bob_r = bob_attach_wrld - bob_.GetPosition();

        vec3 vel_at_pivot = pivot_.GetLinearVelocity() + cross(pivot_.GetAngularVelocity(), pivot_r);
        vec3 vel_at_bob = bob_.GetLinearVelocity() + cross(bob_.GetAngularVelocity(), bob_r);

        float relative_vel_along_rod = dot(vel_at_bob - vel_at_pivot, n);

        float positional_error = l - rod_lenght_;

        //static const float kMaxBias = 10.0f;
        float bias_vel = kConstraintStiffness * positional_error / dt;

        float k_trans = 1.0f / pivot_.GetMass() + 1.0f / bob_.GetMass();

        vec3 r_p_cross_n = cross(pivot_r, n);
        vec3 r_b_cross_n = cross(bob_r, n);

        vec3 pivot_rot_resist = inverse(pivot_.GetInertiaTensor()) * r_p_cross_n;
        vec3 bob_rot_resist   = inverse(bob_.GetInertiaTensor()) * r_b_cross_n;

        float k_rot_pivot = dot(r_p_cross_n, pivot_rot_resist);
        float k_rot_bob   = dot(r_b_cross_n, bob_rot_resist);

        // k is the effective mass of the rod constraint along its direction.
        // A scalar that tells us how "hard" it is to move the pivot and bob
        // along the rod.
        float k = k_trans + k_rot_pivot + k_rot_bob;
        if (k < eps) {
                return;
        }

        // This is the magnitude of the impulse along the rod direction n
        // needed to correct both the current velocity and positional error.
        //
        // Also known as P
        float P = - (relative_vel_along_rod + bias_vel) / k;
        vec3 J = n * P;

        // Apply P to the linear and angular velocities of both bodies
        pivot_.IntegrateLinearImpulse(-J);
        bob_.IntegrateLinearImpulse(J);

        pivot_.IntegrateAngularImpulse(-J, pivot_r);
        bob_.IntegrateAngularImpulse(J, bob_r);

        bob_.IntegrateVelocities(dt);
}

void Pendulum::Draw(wgl::Renderer &renderer)
{
        pivot_.Draw(renderer);
        bob_.Draw(renderer);
}
