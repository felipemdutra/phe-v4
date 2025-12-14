#ifndef ICOSPHERE_H
#define ICOSPHERE_H

#include <cmath>
#include <cstddef>
#include <glm/ext/vector_float3.hpp>
#include <vector>
#include <wrapgl/wrapgl.h>
#include <memory>

typedef unsigned int u32;
typedef long unsigned int u64;
typedef short unsigned int u16;

class IcoSphere {
        std::unique_ptr<wgl::Mesh> mesh_;

        // Position and color.
        std::vector<float> vertices_;
        std::vector<unsigned int> indices_;

        unsigned int dots_vao_, dots_vbo_;
        unsigned int wireframe_vao_, wireframe_vbo_;

        GLsizeiptr edge_vertex_count_;

        void SetupVerticesVao(wgl::VertexLayout layout, float radius, u32 nsubdivisions);
        void SetupDotsVao();
        void SetupWireframeVao();

        void UpdateVerticesVbo(const std::vector<glm::vec3> &new_vertex_positions);
        void UpdateDotsVbo(const std::vector<glm::vec3> &new_vertex_positions);
        void UpdateWireframeVbo(const std::vector<glm::vec3> &new_vertex_positions);

public:
        IcoSphere();

        IcoSphere(wgl::VertexLayout layout, float radius, unsigned int n);
        ~IcoSphere();

        void Draw(wgl::Renderer &renderer);

        // @brief Returns the vertices vectors, which includes position and
        //        color values, in the following layout:
        //
        //        x, y, z, r, g, b,
        //        
        //        so 1 vertex is composed of 6 floats inside the vector.
        std::vector<float>& GetVertices();
        const std::vector<u32>& GetIndices() const { return indices_; }

        std::vector<glm::vec3>& GetPointPositions();
        inline size_t GetPointCount() const { return vertices_.size() / 6; };

        // @brief Returns a vector of ivec3's that contain the position of
        // each triangle of the Ico Sphere.
        std::vector<glm::ivec3> GetTriangles() const;

        // @brief Updates the Ico Sphere's vertex positions with the new
        //        positions.
        // 
        // Updates the Ico Sphere's vertices with the new positions, and
        // updates the vertex buffers, so it draws correctly with the new
        // vertices.
        //
        // @note The @param new_vertex_positions shouldn't contain color 
        //       values, just vec3s with x, y and z values.
        void UpdateVertexPositions(const std::vector<glm::vec3> &new_vertex_positions);
};

#endif

