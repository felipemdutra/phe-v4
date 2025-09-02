#include "../include/spring.h"
#include "../include/point.h"

Spring::Spring(Point *a, Point *b)
{
        this->a = a;
        this->b = b;

        rest_length = glm::length(a->position - b->position);
}

