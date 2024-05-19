#ifndef __POINT_H
#define __POINT_H

#include <cstdint>
#include <cmath>

#define POINT_EPSILON 1e-1
#define ANGLE_EPSILON 1e-3

struct Point2D {
    float x;
    float z;

    bool operator == (const Point2D& p) {
        return std::abs(x - p.x) < POINT_EPSILON && std::abs(z - p.z) < POINT_EPSILON;
    }
};

struct Point3D {
    float x;
    float y;
    float z;
};

#endif