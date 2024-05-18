#ifndef __RASP_H__
#define __RASP_H__

#include <cstdint>

#define RASP_UART_BAUD 115200
#define RX_PIN 16
#define TX_PIN 17

typedef struct {
    uint8_t cmd;
    float dest_x;
    float dest_y;
    float angle;
} RaspRecvData;

typedef struct {
    uint8_t stat;
    float loc_x;
    float loc_y;
} RaspSendData;

void raspRecvTask(void *parameter);

void raspSendTask(void *parameter);

#endif
