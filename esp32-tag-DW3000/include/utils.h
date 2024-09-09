#ifndef __UTILS_H
#define __UTILS_H

#include "point.h"

float getAngle(Point2D target, Point2D cur);

float getAngleDiff(float dest_angle, float tag_angle);

uint8_t calculateCRC(const uint8_t* data, size_t length);

#endif
