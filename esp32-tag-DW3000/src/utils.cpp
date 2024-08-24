#include "utils.h"

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
