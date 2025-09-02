#include "../include/soft_body.h"

#include "../include/util/util.h"
#include <cstdlib>
#include <glm/ext/matrix_transform.hpp>

SoftBody::SoftBody(const glm::vec3 &position, float point_mass, double k) :
        mesh_(IcoSphere(GetGlobalVertexLayout(), 5.0f, 1)), kStiffness(k)
{
        points_.resize(mesh_.GetPointCount(), Point());
        springs_.reserve(mesh_.GetPointCount() * 6);

        auto vertex_positions = mesh_.GetVertices();

        for (size_t i = 0; i < points_.size(); ++i) {
                points_[i] = Point(vertex_positions[i] + position, point_mass);
        }

        for (size_t i = 0; i < points_.size(); ++i) {
                for (size_t j = 0; j < points_.size(); ++j) {
                        if (i == j) {
                                continue;
                        }

                        springs_.push_back(Spring(&points_[i], &points_[j]));
                }
        }

        dirty_ = true;

        UpdateIcoSphereWithPointPositions();
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

void SoftBody::ApplyGravity(double g, float dt)
{
        for (auto p : points_) {
                p.force_accumulator += p.mass * glm::vec3(0.0f, -g, 0.0f);
                (void)dt; // just stop the warning while the function isn't done.
        }
}

void SoftBody::Draw(wgl::Renderer &renderer)
{
        mesh_.Draw(renderer);
}

