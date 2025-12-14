#ifndef SPRING_H
#define SPRING_H

#include <glm/geometric.hpp>
#include <random>

#include "point.h"

struct Spring {
        Point *a = nullptr;
        Point *b = nullptr;

        float rest_length;

        Spring(Point *a, Point *b);
        Spring() = default;

        float GetCurrentLenght() { return fabs(glm::length(a->position - b->position)); }
};

#endif
