#ifndef POINT_H
#define POINT_H

#include <glm/glm.hpp>

struct Point {
        float mass;

        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 force_accumulator;

        Point(glm::vec3 position, float mass) :
                mass(mass),
                position(position),
                velocity({ 0.0f, 0.0f, 0.0f }),
                force_accumulator({ 0.0f, 0.0f, 0.0f }) { }

        Point() : mass(0.0f), position({ 0.0f, 0.0f, 0.0f }),
                  velocity({ 0.0f, 0.0f, 0.0f }),
                  force_accumulator({ 0.0f, 0.0f, 0.0f }) { }
};

#endif

