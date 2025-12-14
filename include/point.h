#ifndef POINT_H
#define POINT_H

#include <complex>
#include <glm/glm.hpp>
#include <vector>

struct Spring;

struct Point {
        float mass;
        size_t i;

        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 force_accumulator;

        std::vector<size_t> spring_indices;
        std::vector<glm::ivec3> triangles;

        Point(size_t i, glm::vec3 position, float mass) :
                mass(mass),
                i(i),
                position(position),
                velocity({ 0.0f, 0.0f, 0.0f }),
                force_accumulator({ 0.0f, 0.0f, 0.0f }) { }

        Point() : mass(0.0f), i(0), position({ 0.0f, 0.0f, 0.0f }),
                  velocity({ 0.0f, 0.0f, 0.0f }),
                  force_accumulator({ 0.0f, 0.0f, 0.0f }) { }

        inline bool operator==(const Point &other) const
        {
                return this->i == other.i;
        }
};

#endif

