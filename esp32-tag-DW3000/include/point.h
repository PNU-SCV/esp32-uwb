#ifndef __POINT_H
#define __POINT_H

#include <cstdint>
#include <cmath>

#define POINT_EPSILON 2e-1
#define ANGLE_EPSILON 20.0f

struct Point2D {
    float x;
    float z;

    bool operator == (const Point2D& p) {
        return pow(pow(x - p.x, 2.0) + pow(z - p.z, 2.0), 0.5) < POINT_EPSILON;
    }
};

struct Point3D {
    float x;
    float y;
    float z;
};

#endif
