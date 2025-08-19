#include "../../include/math/math_utils.h"

glm::mat3 GetCubeInertiaTensor(float mass, float w, float h, float d) {
    float ix = (mass / 12.0f) * (h*h + d*d);
    float iy = (mass / 12.0f) * (w*w + d*d);
    float iz = (mass / 12.0f) * (w*w + h*h);
    static glm::mat3 tensor = glm::mat3(
        ix, 0, 0,
        0, iy, 0,
        0, 0, iz
    );

    return tensor; 
}

glm::mat3 GetSquarePyramidInertiaTensor(float mass, float b, float h) {
    float i_xx = (mass / 20.0f) * (4.0f * h * h + b * b);
    float i_yy = i_xx;
    float i_zz = (mass / 10.0f) * (b * b);
    
    static glm::mat3 tensor = glm::mat3(
        i_xx, 0.0f, 0.0f,
        0.0f, i_yy, 0.0f,
        0.0f, 0.0f, i_zz
    );

    return tensor;
}

