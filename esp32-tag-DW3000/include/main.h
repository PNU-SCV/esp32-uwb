#ifndef MAIN_H
#define MAIN_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define RASP_UART_NUM UART_NUM_1
#define STM32_UART_NUM UART_NUM_2

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;
extern SemaphoreHandle_t stm32_send_flag_semaphore;
extern TaskHandle_t RTLS_task_handle;

#endif 