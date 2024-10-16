#include <Arduino.h>
#include <cmath>
#include "utils.h"

// return : Angle between target & cur vector (clockwise with cur vector)
float getAngle(Point2D target, Point2D cur) 
{
    float dx = target.x - cur.x;
    float dz = target.z - cur.z;

    float angle = atan2f(dz, dx) * 180.0f / PI;
    angle = fmodf(angle + 360.0f, 360.0f);
    return angle;
}


float getAngleDiff(float dest_angle, float tag_angle)
{
    float diff = dest_angle - tag_angle;
    diff = fmodf(diff + 360.0f, 360.0f);
    return diff;
}

uint8_t calculateCRC(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00; 

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; 

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; 
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
