#include "../include/soft_body.h"

#include <cassert>
#include <cstdlib>
#include <glm/detail/qualifier.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/vector_int3.hpp>
#include <glm/geometric.hpp>
#include <unordered_set>
#include <vector>

#include "../include/util/util.h"
#include "../include/math/constants.h"

const glm::vec3 kGravityVec = glm::vec3(0.0f, -kGravity, 0.0f);

static u64 GetPrimePower(u64 a, u64 b)
{
    // Ensure a < b for consistent ordering
    if (a > b) std::swap(a, b);
    return ((u64)a << 32) | b;
}

static float CalculateTriangleArea(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c)
{
        glm::vec3 ab_edge = b - a;
        glm::vec3 ac_edge = c - a;

        auto cross = glm::cross(ab_edge, ac_edge);

        float area = (1.0f / 2.0f) * glm::length(cross);
        return area;
}

static glm::vec3 CalculateTriangleNormal(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c)
{
        glm::vec3 ab_edge = b - a;
        glm::vec3 ac_edge = c - a;
        glm::vec3 cross = glm::cross(ab_edge, ac_edge);

        if (glm::length(cross) < 1e-6f) {
                return glm::vec3(0.0f); // Degenerate triangle
        }

        return glm::normalize(cross); // Don't flip based on center - keep consistent with volume calc
}

glm::vec3 SoftBody::GetCenterPos(const std::vector<State> &states)
{
        glm::vec3 p(0.0f);

        for (auto &state : states) {
                p += state.x;
        }

        return p / (float) states.size();
}

float SoftBody::CalculateVolume(const std::vector<State> &states)
{
        std::vector<glm::ivec3> triangles = mesh_.GetTriangles();
        glm::vec3 center = GetCenterPos(states);

        float per_triangle_volume_contrib = 0.0f;
        for (const auto& t : triangles) {
                auto p0 = states[points_[t.x].i].x - center;
                auto p1 = states[points_[t.y].i].x - center;
                auto p2 = states[points_[t.z].i].x - center;
                per_triangle_volume_contrib += glm::dot(p0, glm::cross(p1, p2));
        }

        return (1.0f / 6.0f) * per_triangle_volume_contrib;
}

float SoftBody::CalculateVolume()
{
        std::vector<glm::ivec3> triangles = mesh_.GetTriangles();

        float per_triangle_volume_contrib = 0.0f;

        for (const auto& t : triangles) {
                auto p0 = points_[t.x].position;
                auto p1 = points_[t.y].position;
                auto p2 = points_[t.z].position;
                per_triangle_volume_contrib += glm::dot(p0, glm::cross(p1, p2));
        }

        return (1.0f / 6.0f) * per_triangle_volume_contrib;
        
}

SoftBody::SoftBody(const glm::vec3 &position, float point_mass, float k) :
        mesh_(IcoSphere(GetGlobalVertexLayout(), 5.0f, 0)), kElasticConstant(k)
{
        points_.resize(mesh_.GetPointCount());
        springs_.reserve(mesh_.GetPointCount() * 6);

        auto vertex_positions = mesh_.GetVertices();

        std::vector<glm::ivec3> triangles = mesh_.GetTriangles();

        for (size_t i = 0; i < points_.size(); ++i) {
                glm::vec3 vertex_pos(vertex_positions[i * 6 + 0], vertex_positions[i * 6 + 1], vertex_positions[i * 6 + 2]);

                points_[i] = Point(i, vertex_pos + position, point_mass);

                for (const auto &t : triangles)  {
                        Point *a = &points_[t.x];
                        Point *b = &points_[t.y];
                        Point *c = &points_[t.z];

                        if (*a == points_[i] || *b == points_[i] || *c == points_[i]) {
                                points_[i].triangles.push_back(t);
                        }
                }
        }

        // Create springs.
        SetupSprings();

        rest_volume_ = CalculateVolume();
        
        dirty_ = true;

        UpdateIcoSphereWithPointPositions();
}

void SoftBody::SetupSprings()
{
        std::vector<glm::ivec3> triangles = mesh_.GetTriangles();
        std::vector<size_t> spring_indices;
        std::unordered_set<u64> seen; // Store indices instead of springs

        for (const auto &t : triangles) {
                Point *a = &points_[t.x];
                Point *b = &points_[t.y];
                Point *c = &points_[t.z];

                u64 ab = GetPrimePower(t.x, t.y);
                u64 ac = GetPrimePower(t.x, t.z);
                u64 bc = GetPrimePower(t.y, t.z);

                if (seen.find(ab) == seen.end()) {
                        springs_.emplace_back(a, b);
                        seen.insert(ab);
                }

                if (seen.find(ac) == seen.end()) {
                        springs_.emplace_back(a, c);
                        seen.insert(ac);
                }

                if (seen.find(bc) == seen.end()) {
                        springs_.emplace_back(b, c);
                        seen.insert(bc);
                }
        }
}

SoftBody::~SoftBody()
{

}

void SoftBody::UpdateIcoSphereWithPointPositions()
{
        if (!dirty_) return;

        std::vector<glm::vec3> new_positions;
        new_positions.reserve(points_.size());

        for (const auto &p : points_) {
                new_positions.push_back(p.position);
        }
        
        mesh_.UpdateVertexPositions(new_positions);
        
        dirty_ = false;
}

std::vector<State> SoftBody::BuildStates()
{
        std::vector<State> states;
        states.resize(points_.size());

        for (size_t i = 0; i < points_.size(); ++i) {
                states[i] = State { points_[i].position, points_[i].velocity };
        }

        return states;
}

std::vector<StateDerivative> SoftBody::ComputeStatesDerivative(const std::vector<State> &states)
{
        std::vector<StateDerivative> new_state_derivatives; // Derivative of states.
        new_state_derivatives.resize(states.size());

        for (size_t i = 0; i < new_state_derivatives.size(); ++i) {
                new_state_derivatives[i].v = states[i].v;
                new_state_derivatives[i].a = glm::vec3(0.0f, 0.0f, 0.0f);
        }

        // Note that states[i] = new_state_derivatives[i].
        for (auto &p : points_) {
                glm::vec3 gforce = p.mass * kGravityVec;
                
                // Add gforce to acceleration.
                new_state_derivatives[p.i].a += gforce / p.mass;
        }

        const static float kFloorStiffness = kElasticConstant * 100.0f;
        const static float kFloorDamping = kElasticDamping * 2.0f;

        // Ground force
        for (auto &p : points_) {
                new_state_derivatives[p.i].a += (float) -kGlobalDamping * states[p.i].v / p.mass;

                float penetration = kFloorY - states[p.i].x.y;
                if (penetration > 0.0f) {
                        // compute a "soft spring" force
                        float f_spring = kFloorStiffness * penetration;

                        // damping only in y-direction
                        float f_damping = -kFloorDamping * states[p.i].v.y;

                        // total floor force
                        glm::vec3 floor_force(0.0f, f_spring + f_damping, 0.0f);

                        // apply acceleration
                        new_state_derivatives[p.i].a += floor_force / p.mass;

                        // optional: prevent extreme penetration (projection)
                }
        }

        // Calculate volume to find pressure force.
        float volume = CalculateVolume(states);
        const static float min_volume = rest_volume_ * 0.1f; // Don't compress below 10% of rest volume

        volume = std::max(volume, min_volume);

        float pressure = kPressure * ((rest_volume_ - volume) / rest_volume_);

        std::vector<glm::ivec3> triangles = mesh_.GetTriangles();

        for (auto &s : springs_) {
                glm::vec3 delta = s.b->position - s.a->position;
                float length = glm::length(delta);
                glm::vec3 dir = delta / (length + 1e-6f);

                float spring_force_mag   = kElasticConstant * (length - s.rest_length);
                float damping_force_mag  = kElasticDamping * glm::dot(s.b->velocity - s.a->velocity, dir);

                glm::vec3 force = (spring_force_mag + damping_force_mag) * dir;

                new_state_derivatives[s.a->i].a += force / s.a->mass;
                new_state_derivatives[s.b->i].a -= force / s.b->mass;
        }

        for (const auto &t : triangles) {
                Point *a = &points_[t.x];
                Point *b = &points_[t.y];
                Point *c = &points_[t.z];

                glm::vec3 apos = states[a->i].x;
                glm::vec3 bpos = states[b->i].x;
                glm::vec3 cpos = states[c->i].x;

                // Compute triangle area and outward normal
                glm::vec3 normal = CalculateTriangleNormal(apos, bpos, cpos);
                float area = CalculateTriangleArea(apos, bpos, cpos);

                // Total force exerted by pressure on this triangle
                glm::vec3 pressuref =
                        pressure * area * normal;

                // Distribute evenly to the 3 vertices
                glm::vec3 appliedf = pressuref / 3.0f;

                new_state_derivatives[a->i].a += appliedf / a->mass;
                new_state_derivatives[b->i].a += appliedf / b->mass;
                new_state_derivatives[c->i].a += appliedf / c->mass;
        }

        return new_state_derivatives;
}


void SoftBody::Update(float dt)
{
        if (dt > 0.008) {
                dt = 0.008;
        }

        auto curr_states = BuildStates();

        assert(curr_states.size() == points_.size());

        std::vector<StateDerivative> k1;
        k1 = ComputeStatesDerivative(curr_states);

        auto temp = curr_states;
        for (size_t i = 0; i < temp.size(); ++i) {
                temp[i].x += k1[i].v * dt / 2.0f;
                temp[i].v += k1[i].a * dt / 2.0f;

        }

        std::vector<StateDerivative> k2;
        k2 = ComputeStatesDerivative(temp);

        temp = curr_states;
        for (size_t i = 0; i < temp.size(); ++i) {
                temp[i].x += k2[i].v * dt / 2.0f;
                temp[i].v += k2[i].a * dt / 2.0f;
        }

        std::vector<StateDerivative> k3;
        k3 = ComputeStatesDerivative(temp);

        temp = curr_states;
        for (size_t i = 0; i < temp.size(); ++i) {
                temp[i].x += k3[i].v * dt;
                temp[i].v += k3[i].a * dt;
        }

        std::vector<StateDerivative> k4;
        k4 = ComputeStatesDerivative(temp);

        for (auto &p : points_) {
                glm::vec3 avrg_pos(dt / 6.0f * (k1[p.i].v + 2.0f * k2[p.i].v + 2.0f * k3[p.i].v + k4[p.i].v));
                glm::vec3 avrg_vel(dt / 6.0f * (k1[p.i].a + 2.0f * k2[p.i].a + 2.0f * k3[p.i].a + k4[p.i].a));

                p.position += avrg_pos;
                p.velocity += avrg_vel;
        }

        dirty_ = true;

        UpdateIcoSphereWithPointPositions();
}

void SoftBody::Draw(wgl::Renderer &renderer)
{
        mesh_.Draw(renderer);
}

