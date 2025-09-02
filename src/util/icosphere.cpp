#include "../../include/util/icosphere.h"

#include <cstdint>
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <unordered_map>
#include <vector>
#include <wrapgl/renderer.h>

using std::vector;

static inline uint64_t edge_key(unsigned int a, unsigned int b) {
        if (a < b) return (uint64_t(a) << 32) | uint64_t(b);
        return (uint64_t(b) << 32) | uint64_t(a);
}

IcoSphere::IcoSphere(wgl::VertexLayout layout, float radius, unsigned int n)
{
        const float phi = (1.0f + sqrt(5.0f)) / 2.0f; // Golden ratio
        
        vector<glm::vec3> vertices = {
                // Group 1: (0, ±1, ±φ)
                { 0.0f,  1.0f,  phi },  // 0
                { 0.0f, -1.0f,  phi },  // 1  
                { 0.0f,  1.0f, -phi },  // 2
                { 0.0f, -1.0f, -phi },  // 3

                // Group 2: (±1, ±φ, 0)
                { 1.0f,  phi, 0.0f },   // 4
                {-1.0f,  phi, 0.0f },   // 5
                { 1.0f, -phi, 0.0f },   // 6
                {-1.0f, -phi, 0.0f },   // 7

                // Group 3: (±φ, 0, ±1)
                { phi, 0.0f,  1.0f },   // 8
                {-phi, 0.0f,  1.0f },   // 9
                { phi, 0.0f, -1.0f },   // 10
                {-phi, 0.0f, -1.0f }    // 11
        }; 

        // Normalize each vertex.
        for (auto &vertex : vertices) {
                vertex = glm::normalize(vertex) * radius;
        }

        // Corrected triangle indices with consistent counter-clockwise winding
        vector<unsigned int> indices = {
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

        for (unsigned int i = 0; i < n; ++i) {
                vector<unsigned int> new_indices;
                new_indices.reserve(indices.size() * 4);

                std::unordered_map<uint64_t, unsigned int> midpoint_cache;
                midpoint_cache.reserve(indices.size() / 2);

                auto get_midpoint = [&](unsigned int ia, unsigned int ib) -> unsigned int {
                        uint64_t key = edge_key(ia, ib);
                        auto it = midpoint_cache.find(key);

                        if  (it != midpoint_cache.end()) {
                                return it->second;
                        }

                        // Calculate midpoint, normalize to sphere surface
                        glm::vec3 mid = (vertices[ia] + vertices[ib]) * 0.5f;
                        mid = glm::normalize(mid) * radius;
                        
                        unsigned int mid_index = (unsigned int)vertices.size();
                        vertices.push_back(mid);
                        midpoint_cache[key] = mid_index;

                        return mid_index;
                };

                // For each triangle, create 4 new triangles using midpoints
                for (size_t i = 0; i < indices.size(); i += 3) {
                        unsigned int ia = indices[i];
                        unsigned int ib = indices[i + 1];
                        unsigned int ic = indices[i + 2];

                        unsigned int m_ab = get_midpoint(ia, ib);
                        unsigned int m_bc = get_midpoint(ib, ic);
                        unsigned int m_ca = get_midpoint(ic, ia);

                        // Create 4 new triangles, preserving winding order
                        new_indices.push_back(ia);
                        new_indices.push_back(m_ab);
                        new_indices.push_back(m_ca);

                        new_indices.push_back(ib);
                        new_indices.push_back(m_bc);
                        new_indices.push_back(m_ab);

                        new_indices.push_back(ic);
                        new_indices.push_back(m_ca);
                        new_indices.push_back(m_bc);

                        new_indices.push_back(m_ab);
                        new_indices.push_back(m_bc);
                        new_indices.push_back(m_ca);
                }

                indices.swap(new_indices);
        }

        vector<float> new_vertices;

        for (const auto &v : vertices) {
                new_vertices.push_back(v.x);
                new_vertices.push_back(v.y);
                new_vertices.push_back(v.z);

                new_vertices.push_back(1.0f);
                new_vertices.push_back(0.0f);
                new_vertices.push_back(0.0f);
        }

        vertices_ = new_vertices;
        indices_ = indices;

        mesh_ = std::make_unique<wgl::Mesh>(layout, vertices_, indices_, true);

        // Build VBO for black dots
        vector<float> dot_vertices;
        dot_vertices.reserve(vertices.size() * 6); // position + color

        for (const auto &v : vertices) {
                dot_vertices.push_back(v.x);
                dot_vertices.push_back(v.y);
                dot_vertices.push_back(v.z);

                dot_vertices.push_back(1.0f); // R
                dot_vertices.push_back(1.0f); // G
                dot_vertices.push_back(1.0f); // B
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

        // Build VBO for wireframe edges
        vector<float> edge_vertices;
        edge_vertices.reserve(indices.size() * 2 * 6); // 2 vertices per edge, 6 floats per vertex (pos + color)

        for (size_t i = 0; i < indices.size(); i += 3) {
                unsigned int ia = indices[i];
                unsigned int ib = indices[i + 1]; 
                unsigned int ic = indices[i + 2];

                // Get vertex positions (remembering vertices are now in the flattened array)
                glm::vec3 va(new_vertices[ia * 6], new_vertices[ia * 6 + 1], new_vertices[ia * 6 + 2]);
                glm::vec3 vb(new_vertices[ib * 6], new_vertices[ib * 6 + 1], new_vertices[ib * 6 + 2]);
                glm::vec3 vc(new_vertices[ic * 6], new_vertices[ic * 6 + 1], new_vertices[ic * 6 + 2]);

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

        glBufferData(GL_ARRAY_BUFFER, edge_vertex_size * edge_vertex_count, edge_vertices.data(), GL_STATIC_DRAW);

        // Wireframe position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, edge_vertex_size, (void*)0);

        // Wireframe color 
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, edge_vertex_size, (void*)(sizeof(float) * 3));

        glBindVertexArray(0);
}

void IcoSphere::UpdateVertexBuffer() const
{
        mesh_->UpdateVertexBuffer(vertices_);
}

void IcoSphere::
UpdateVertexPositions(const std::vector<glm::vec3> &new_vertex_positions)
{
        for (size_t i = 0; i < vertices_.size(); i += 6) {
                vertices_[i+0] = new_vertex_positions[i/6].x;
                vertices_[i+1] = new_vertex_positions[i/6].y;
                vertices_[i+2] = new_vertex_positions[i/6].z;
        }

        UpdateVertexBuffer();

        // Update dots VBO
        std::vector<float> dot_vertices;
        dot_vertices.reserve(new_vertex_positions.size() * 6);

        for (const auto &v : new_vertex_positions) {
                dot_vertices.push_back(v.x);
                dot_vertices.push_back(v.y);
                dot_vertices.push_back(v.z);
                dot_vertices.push_back(1.0f); // R
                dot_vertices.push_back(1.0f); // G
                dot_vertices.push_back(1.0f); // B
        }

        glBindBuffer(GL_ARRAY_BUFFER, dots_vbo_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, dot_vertices.size() * sizeof(float), dot_vertices.data());

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
        glBufferSubData(GL_ARRAY_BUFFER, 0, edge_vertices.size() * sizeof(float), edge_vertices.data());

        glBindBuffer(GL_ARRAY_BUFFER, 0);
}

std::vector<float>& IcoSphere::GetVertices()
{
        return vertices_;
}

void IcoSphere::Draw(wgl::Renderer &renderer)
{ 
        mesh_->Draw(renderer);

        glPointSize(5.0f);
        renderer.DrawArrays(dots_vao_, GL_POINTS, static_cast<GLsizei>(vertices_.size() / 6));

        glLineWidth(3.0f);
        renderer.DrawArrays(wireframe_vao_, GL_LINES, edge_vertex_count_);
}

IcoSphere::~IcoSphere()
{

}

