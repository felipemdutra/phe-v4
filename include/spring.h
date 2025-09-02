#ifndef SPRING_H
#define SPRING_H

#include "point.h"

struct Spring {
        Point *a;
        Point *b;

        float rest_length;

        double spring_stiffness;
        double spring_damping_factor;

        Spring(Point *a, Point *b);
};

#endif
