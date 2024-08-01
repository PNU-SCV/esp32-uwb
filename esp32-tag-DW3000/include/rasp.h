#ifndef __RASP_H__
#define __RASP_H__

#include "point.h"

#define RASP_UART_BAUD 115200
#define RASP_RX_PIN 39
#define RASP_TX_PIN 26

typedef struct {
    uint8_t cmd;
    float dest_x;
    float dest_z;
    float angle;
} RaspRecvData;

typedef struct {
    uint8_t stat;
    float loc_x;
    float loc_z;
} RaspSendData;

void raspRecvTask(void *parameter);

void raspSendTask(void *parameter);

#endif
