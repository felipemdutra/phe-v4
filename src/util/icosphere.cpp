#include "../../include/util/icosphere.h"

#include <cstdint>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

using std::vector;

const float PHI = (1.0f + sqrt(5.0f)) / 2.0f; // Golden ratio

static inline uint64_t edge_key(unsigned int a, unsigned int b) {
        if (a < b) return (uint64_t(a) << 32) | uint64_t(b);
        return (uint64_t(b) << 32) | uint64_t(a);
}

std::vector<glm::ivec3> IcoSphere::GetTriangles() const
{
        auto indices = GetIndices();

        std::vector<glm::ivec3> triangles;
        triangles.reserve(indices.size());

        for (size_t i = 0; i < indices.size(); i += 3) {
                triangles.push_back({ indices[i+0], indices[i+1], indices[i+2] });
        }

        return triangles;
}

void IcoSphere::SetupVerticesVao(wgl::VertexLayout layout, float radius, u32 nsubdivisions)
{
        vector<glm::vec3> identity_vertices = {
                { 0.0f,  1.0f,  PHI },
                { 0.0f, -1.0f,  PHI },
                { 0.0f,  1.0f, -PHI },
                { 0.0f, -1.0f, -PHI },

                { 1.0f,  PHI, 0.0f },
                {-1.0f,  PHI, 0.0f },
                { 1.0f, -PHI, 0.0f },
                {-1.0f, -PHI, 0.0f },

                { PHI, 0.0f,  1.0f },
                {-PHI, 0.0f,  1.0f },
                { PHI, 0.0f, -1.0f },
                {-PHI, 0.0f, -1.0f }
        }; 

        // Normalize each vertex.
        for (auto &vertex : identity_vertices) {
                vertex = glm::normalize(vertex) * radius;
        }

        vector<unsigned int> identity_indices = {
                // Triangles around vertex 0 (top front)
                0, 8, 4,
                0, 4, 5,
                0, 5, 9,
                0, 9, 1,
                0, 1, 8,
                
                // Triangles around vertex 2 (top back)  
                2, 4, 10,
                2, 10, 11,
                2, 11, 5,
                2, 5, 4,
                
                // Triangles around vertex 1 (bottom front)
                1, 9, 7,
                1, 7, 6,
                1, 6, 8,
                
                // Triangles around vertex 3 (bottom back)
                3, 11, 10,
                3, 10, 6,
                3, 6, 7,
                3, 7, 11,
                
                // Middle belt triangles
                4, 8, 10,
                6, 10, 8,
                5, 11, 9,
                7, 9, 11
        };

        for (unsigned int i = 0; i < nsubdivisions; ++i) {
                vector<unsigned int> indices;
                indices.reserve(identity_indices.size() * 4);

                std::unordered_map<uint64_t, unsigned int> midpoint_cache;
                midpoint_cache.reserve(indices.size() / 2);

                auto get_midpoint = [&](unsigned int ia, unsigned int ib) -> unsigned int {
                        uint64_t key = edge_key(ia, ib);
                        auto it = midpoint_cache.find(key);

                        if  (it != midpoint_cache.end()) {
                                return it->second;
                        }

                        // Calculate midpoint, normalize to sphere surface
                        glm::vec3 mid = (identity_vertices[ia] + identity_vertices[ib]) * 0.5f;
                        mid = glm::normalize(mid) * radius;
                        
                        unsigned int mid_index = (unsigned int)identity_vertices.size();
                        identity_vertices.push_back(mid);
                        midpoint_cache[key] = mid_index;

                        return mid_index;
                };

                // For each triangle, create 4 new triangles using midpoints
                for (size_t i = 0; i < identity_indices.size(); i += 3) {
                        unsigned int ia = identity_indices[i];
                        unsigned int ib = identity_indices[i + 1];
                        unsigned int ic = identity_indices[i + 2];

                        unsigned int m_ab = get_midpoint(ia, ib);
                        unsigned int m_bc = get_midpoint(ib, ic);
                        unsigned int m_ca = get_midpoint(ic, ia);

                        // Create 4 new triangles, preserving winding order
                        indices.push_back(ia);
                        indices.push_back(m_ab);
                        indices.push_back(m_ca);

                        indices.push_back(ib);
                        indices.push_back(m_bc);
                        indices.push_back(m_ab);

                        indices.push_back(ic);
                        indices.push_back(m_ca);
                        indices.push_back(m_bc);

                        indices.push_back(m_ab);
                        indices.push_back(m_bc);
                        indices.push_back(m_ca);
                }

                identity_indices.swap(indices);
        }

        vector<float> vertices;

        for (const auto &v : identity_vertices) {
                vertices.push_back(v.x);
                vertices.push_back(v.y);
                vertices.push_back(v.z);

                vertices.push_back(1.0f);
                vertices.push_back(0.0f);
                vertices.push_back(0.0f);
        }

        vertices_ = vertices;
        indices_ = identity_indices;

        mesh_ = std::make_unique<wgl::Mesh>(layout, vertices_, indices_, true);

}

void IcoSphere::SetupDotsVao()
{
        // Build VBO for white dots
        vector<float> dot_vertices;
        dot_vertices.reserve(vertices_.size() * 6); // position + color

        for (size_t i = 0; i < vertices_.size(); i += 6) {
                // Position.
                dot_vertices.push_back(vertices_[i + 0]);
                dot_vertices.push_back(vertices_[i + 1]);
                dot_vertices.push_back(vertices_[i + 2]);

                // Color.
                dot_vertices.push_back(1.0f);
                dot_vertices.push_back(1.0f);
                dot_vertices.push_back(1.0f);
        }

        glGenVertexArrays(1, &dots_vao_);
        glGenBuffers(1, &dots_vbo_);

        glBindVertexArray(dots_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, dots_vbo_);

        const GLsizeiptr vertex_size  = sizeof(float) * 6;
        const GLsizeiptr vertex_count = dot_vertices.size() / 6;

        glBufferData(GL_ARRAY_BUFFER, vertex_size * vertex_count, dot_vertices.data(), GL_DYNAMIC_DRAW);

        // Dots position.
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, vertex_size, (void*)0);

        // Dots color.
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, vertex_size, (void*)(sizeof(float) * 3));

        glBindVertexArray(0);
}

void IcoSphere::SetupWireframeVao()
{
        // Build VBO for wireframe edges
        vector<float> edge_vertices;
        edge_vertices.reserve(indices_.size() * 2 * 6); // 2 vertices per edge, 6 floats per vertex (pos + color)

        for (size_t i = 0; i < indices_.size(); i += 3) {
                unsigned int ia = indices_[i];
                unsigned int ib = indices_[i + 1]; 
                unsigned int ic = indices_[i + 2];

                // Get vertex positions (remembering vertices are now in the flattened array)
                glm::vec3 va(vertices_[ia * 6], vertices_[ia * 6 + 1], vertices_[ia * 6 + 2]);
                glm::vec3 vb(vertices_[ib * 6], vertices_[ib * 6 + 1], vertices_[ib * 6 + 2]);
                glm::vec3 vc(vertices_[ic * 6], vertices_[ic * 6 + 1], vertices_[ic * 6 + 2]);

                // Edge A->B
                edge_vertices.insert(edge_vertices.end(), {va.x, va.y, va.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {vb.x, vb.y, vb.z, 0.0f, 0.0f, 0.0f});

                // Edge B->C  
                edge_vertices.insert(edge_vertices.end(), {vb.x, vb.y, vb.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {vc.x, vc.y, vc.z, 0.0f, 0.0f, 0.0f});

                // Edge C->A
                edge_vertices.insert(edge_vertices.end(), {vc.x, vc.y, vc.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {va.x, va.y, va.z, 0.0f, 0.0f, 0.0f});
        }

        glGenVertexArrays(1, &wireframe_vao_);
        glGenBuffers(1, &wireframe_vbo_);

        glBindVertexArray(wireframe_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, wireframe_vbo_);

        const GLsizeiptr edge_vertex_size = sizeof(float) * 6;
        const GLsizeiptr edge_vertex_count = edge_vertices.size() / 6;

        edge_vertex_count_ = edge_vertex_count;

        glBufferData(GL_ARRAY_BUFFER, edge_vertex_size * edge_vertex_count, edge_vertices.data(), GL_DYNAMIC_DRAW);

        // Wireframe position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, edge_vertex_size, (void*)0);

        // Wireframe color 
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, edge_vertex_size, (void*)(sizeof(float) * 3));

        glBindVertexArray(0);
}

IcoSphere::IcoSphere(wgl::VertexLayout layout, float radius, unsigned int n)
{
        // Do not change order of setup!
        SetupVerticesVao(layout, radius, n);

        SetupDotsVao();

        SetupWireframeVao();
}


void IcoSphere::UpdateVerticesVbo(const std::vector<glm::vec3> &new_vertex_positions)
{
        assert(new_vertex_positions.size() == GetPointCount());
        for (size_t i = 0; i < vertices_.size(); i += 6) {
                vertices_[i+0] = new_vertex_positions[i/6].x;
                vertices_[i+1] = new_vertex_positions[i/6].y;
                vertices_[i+2] = new_vertex_positions[i/6].z;
        }

        mesh_->UpdateVertexBuffer(vertices_);

}

void IcoSphere::UpdateDotsVbo(const std::vector<glm::vec3> &new_vertex_positions)
{
        // Update dots VBO
        vector<float> dot_vertices;
        dot_vertices.reserve(GetPointCount() * 6); // position + color

        for (size_t i = 0; i < GetPointCount(); ++i) {
                // Position.
                dot_vertices.push_back(new_vertex_positions[i].x);
                dot_vertices.push_back(new_vertex_positions[i].y);
                dot_vertices.push_back(new_vertex_positions[i].z);

                // Color.
                dot_vertices.push_back(1.0f);
                dot_vertices.push_back(1.0f);
                dot_vertices.push_back(1.0f);
        }

        const GLsizeiptr vertex_size  = sizeof(float) * 6;
        const GLsizeiptr vertex_count = dot_vertices.size() / 6;

        glBindBuffer(GL_ARRAY_BUFFER, dots_vbo_);
        assert(glGetError() == GL_NO_ERROR);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertex_count * vertex_size, dot_vertices.data());
        assert(glGetError() == GL_NO_ERROR);
}

void IcoSphere::UpdateWireframeVbo(const std::vector<glm::vec3> &new_vertex_positions)
{
        // Update wireframe VBO
        std::vector<float> edge_vertices;
        edge_vertices.reserve(indices_.size() * 2 * 6); // 2 vertices per edge, 6 floats per vertex

        for (size_t i = 0; i < indices_.size(); i += 3) {
                unsigned int ia = indices_[i];
                unsigned int ib = indices_[i + 1];
                unsigned int ic = indices_[i + 2];

                const glm::vec3 &va = new_vertex_positions[ia];
                const glm::vec3 &vb = new_vertex_positions[ib];
                const glm::vec3 &vc = new_vertex_positions[ic];

                // Edge A->B
                edge_vertices.insert(edge_vertices.end(), {va.x, va.y, va.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {vb.x, vb.y, vb.z, 0.0f, 0.0f, 0.0f});

                // Edge B->C
                edge_vertices.insert(edge_vertices.end(), {vb.x, vb.y, vb.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {vc.x, vc.y, vc.z, 0.0f, 0.0f, 0.0f});

                // Edge C->A
                edge_vertices.insert(edge_vertices.end(), {vc.x, vc.y, vc.z, 0.0f, 0.0f, 0.0f});
                edge_vertices.insert(edge_vertices.end(), {va.x, va.y, va.z, 0.0f, 0.0f, 0.0f});
        }

        glBindBuffer(GL_ARRAY_BUFFER, wireframe_vbo_);

        // Check current buffer size
        GLint current_buffer_size;
        glGetBufferParameteriv(GL_ARRAY_BUFFER, GL_BUFFER_SIZE, &current_buffer_size);

        GLint required_size = edge_vertices.size() * sizeof(float);

        if (required_size > current_buffer_size) {
                // Buffer too small, reallocate
                glBufferData(GL_ARRAY_BUFFER, required_size, edge_vertices.data(), GL_DYNAMIC_DRAW);
                // Update the stored count
                edge_vertex_count_ = edge_vertices.size() / 6;
        } else {
                // Buffer is large enough, just update
                glBufferSubData(GL_ARRAY_BUFFER, 0, required_size, edge_vertices.data());
        }

        assert(glGetError() == GL_NO_ERROR);
}

void IcoSphere::
UpdateVertexPositions(const std::vector<glm::vec3> &new_vertex_positions)
{
        UpdateVerticesVbo(new_vertex_positions);

        UpdateDotsVbo(new_vertex_positions);

        UpdateWireframeVbo(new_vertex_positions);
}

std::vector<float>& IcoSphere::GetVertices()
{
        return vertices_;
}

void IcoSphere::Draw(wgl::Renderer &renderer)
{ 
        renderer.SetUniformMatrix4f("model", glm::mat4(1.0f));
        mesh_->Draw(renderer);

        glPointSize(3.0f);
        renderer.DrawArrays(dots_vao_, GL_POINTS, static_cast<GLsizei>(vertices_.size() / 6));

        glLineWidth(3.0f);
        renderer.DrawArrays(wireframe_vao_, GL_LINES, edge_vertex_count_);
}

IcoSphere::~IcoSphere()
{

}

