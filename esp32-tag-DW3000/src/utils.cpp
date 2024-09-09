#include <Arduino.h>
#include <cmath>
#include "utils.h"

float getAngle(Point2D target, Point2D cur) 
{
  float dx = target.x - cur.x;
  float dz = target.z - cur.z;

  return ((int)(std::atan2(dz, dx) * 180.0f / PI) + 360 ) % 360;
}

float getAngleDiff(float dest_angle, float tag_angle)
{
    return (int)(dest_angle - tag_angle + 360) % 360;
}

uint8_t calculateCRC(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00; // 초기 CRC 값

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // 데이터와 CRC를 XOR 연산

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // 다항식 x^8 + x^2 + x + 1 사용
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
