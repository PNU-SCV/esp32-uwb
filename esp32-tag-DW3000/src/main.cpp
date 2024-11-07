/*
 * Copyright 2024 seokwon Yoon
 *
 * This work includes code from the DW3000 library developed by Yannick SILVA.
 * 
 * Author: Yannick SILVA <yannick.silva@gmail.com>
 * Maintainer: Yannick SILVA <yannick.silva@gmail.com>
 * Original source: https://nconcepts.fr
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "main.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "dw3000.h"
#include "DW3000_RTLS.h"
#include "rasp.h"
#include "stm32.h"

/*********************************************************************************************************************************************************
 *                                                      Extern Variables
 *********************************************************************************************************************************************************/

extern uint32_t last_synced_time;

extern dwt_txconfig_t txconfig_options;

/*********************************************************************************************************************************************************
 *                                                      Global Variables
 *********************************************************************************************************************************************************/

TaskHandle_t RTLS_task_handle = NULL;

TaskHandle_t rasp_recv_task_handle = NULL;
TaskHandle_t rasp_send_task_handle = NULL;

TaskHandle_t stm32_recv_task_handle = NULL;
TaskHandle_t stm32_send_task_handle = NULL;

SemaphoreHandle_t stm32_recv_data_semaphore;
SemaphoreHandle_t rasp_recv_data_semaphore;

SemaphoreHandle_t stm32_send_flag_semaphore;

DW3000_RTLS dw3000_rtls;

Point2D tagPosition = {0.0, 0.0};

/*********************************************************************************************************************************************************
 *                                                          RTLS Constructors
 *********************************************************************************************************************************************************/

void RTLSTaskWrapper(void *parameter) 
{
    while(true) 
    {
        dw3000_rtls.RTLSTaskPrologue();

        dw3000_rtls.setLocation();

        dw3000_rtls.RTLSTaskEpilogue();

        tagPosition = dw3000_rtls.getTagPosition();
    }
}

/*********************************************************************************************************************************************************
 *                                                          Constructors
 *********************************************************************************************************************************************************/

void setupUART(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, QueueHandle_t *uart_queue) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    if(uart_queue == NULL) {
        ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 0, 0, 0, 0));
    } else {
        ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 0, 20, uart_queue, 0));
    }

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    if(tx_pin != 0 && rx_pin != 0) {
        ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    }
}

void setup()
{
    esp_log_level_set("*", ESP_LOG_NONE);
    /***************** RTLS Setup Begin *****************/
    
    dw3000_rtls.RTLSSetup();

    ESP_LOGI("MAIN", "RTLS Setup Complete");

    /***************** RTLS Setup End *****************/

    /***************** Rasp Setup Begin *****************/

    QueueHandle_t rasp_uart_queue;
    setupUART(RASP_UART_NUM, RASP_TX_PIN, RASP_RX_PIN, RASP_UART_BAUD, &rasp_uart_queue);

    ESP_LOGI("MAIN", "Raspberry Pi UART Setup Complete");

    /***************** Rasp Setup End *****************/

    /***************** STM32 Setup Begin *****************/

    QueueHandle_t stm32_uart_queue;
    setupUART(STM32_UART_NUM, STM32_TX_PIN, STM32_RX_PIN, STM32_UART_BAUD, &stm32_uart_queue);

    ESP_LOGI("MAIN", "STM32 UART Setup Complete");

    /***************** STM32 Setup End *****************/

    /***************** Semaphore Setup Begin *****************/
    
    stm32_recv_data_semaphore = xSemaphoreCreateBinary();
    rasp_recv_data_semaphore = xSemaphoreCreateBinary();
    stm32_send_flag_semaphore = xSemaphoreCreateBinary();

    xSemaphoreGive(stm32_recv_data_semaphore);
    xSemaphoreGive(rasp_recv_data_semaphore);

    ESP_LOGI("MAIN", "Semaphore Setup Complete");

    /***************** Semaphore Setup End *****************/

    // Create Rasp Tasks
    if (xTaskCreatePinnedToCore(raspRecvTask, "RaspRecvTask", 4096, (void *)rasp_uart_queue, 1, &rasp_recv_task_handle, 0) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create Rasp Recv Task");
    }

    if (xTaskCreatePinnedToCore(raspSendTask, "RaspSendTask", 4096, NULL, 1, &rasp_send_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create Rasp Send Task");
    }

    // Create STM32 Tasks
    if (xTaskCreatePinnedToCore(stm32RecvTask, "STM32RecvTask", 4096, (void *)stm32_uart_queue, 2, &stm32_recv_task_handle, 0) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create STM32 Recv Task");
    }

    if (xTaskCreatePinnedToCore(stm32SendTask, "STM32SendTask", 4096, NULL, 2, &stm32_send_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create STM32 Send Task");
    }

    // Create RTLS Task
    if (xTaskCreatePinnedToCore(RTLSTaskWrapper, "RTLS_Task", 8192, NULL, 3, &RTLS_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create RTLS Task");
    }

    ESP_LOGI("MAIN", "Setup Complete. Tasks Started.");
}

void loop()
{
}
