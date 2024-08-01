#ifndef __STM32_H
#define __STM32_H

#include "point.h"

#define STM32_UART_BAUD 115200
#define STM32_RX_PIN 36
#define STM32_TX_PIN 25

#define CMD_STOP (uint8_t)0x00
#define CMD_FORWARD (uint8_t)0x01
#define CMD_CLOCKWISE_ROTATE (uint8_t)0x02
#define CMD_COUNTERCLOCKWISE_ROTATE (uint8_t)0x03

typedef struct {
    uint8_t status;
} STM32RecvData;

typedef struct {
    uint8_t cmd;
} STM32SendData;

void stm32RecvTask(void *parameter);

void stm32SendTask(void *parameter);

#endif
