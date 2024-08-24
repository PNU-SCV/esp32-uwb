#ifndef __RASP_H__
#define __RASP_H__

#include "point.h"

#define RASP_UART_BAUD 115200
#define RASP_RX_PIN 39
#define RASP_TX_PIN 26

#pragma pack(push, 1)

typedef struct {
    uint8_t cmd;
    float dest_x;
    float dest_z;
    float angle;
    uint8_t crc; 
} RaspRecvData;

typedef struct {
    uint8_t stat;
    float loc_x;
    float loc_z;
    uint8_t crc; 
} RaspSendData;

#pragma pack(pop)


void raspRecvTask(void *parameter);

void raspSendTask(void *parameter);

#endif
