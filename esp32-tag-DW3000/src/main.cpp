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
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
 * 														Extern Variables
 *********************************************************************************************************************************************************/

extern uint32_t last_synced_time;

extern dwt_txconfig_t txconfig_options;

/*********************************************************************************************************************************************************
 * 														Global Variables
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
 * 														    ISR Handlers
 *********************************************************************************************************************************************************/

// UART ISR handler for Raspberry Pi
void IRAM_ATTR onRaspDataAvailable(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(rasp_recv_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// UART ISR handler for STM32
void IRAM_ATTR onStm32DataAvailable(void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(stm32_recv_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/*********************************************************************************************************************************************************
 * 														    RTLS Constructors
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
 * 														    Constructors
 *********************************************************************************************************************************************************/

void setupUART(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, void (*isr_handler)(void*)) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, 2048, 0, 0, NULL, 0);
    uart_isr_register(uart_num, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  // Pass NULL as the last parameter
    uart_enable_rx_intr(uart_num);
}

void setup()
{
    /***************** RTLS Setup Begin *****************/
    
    dw3000_rtls.RTLSSetup();

    ESP_LOGI("MAIN", "RTLS Setup Complete");

    /***************** RTLS Setup End *****************/


    /***************** Rasp Setup Begin *****************/

    setupUART(RASP_UART_NUM, RASP_TX_PIN, RASP_RX_PIN, RASP_UART_BAUD, onRaspDataAvailable);

    ESP_LOGI("MAIN", "Raspberry Pi UART Setup Complete");

    /***************** Rasp Setup End *****************/

    /***************** STM32 Setup Begin *****************/

    setupUART(STM32_UART_NUM, STM32_TX_PIN, STM32_RX_PIN, STM32_UART_BAUD, onStm32DataAvailable);

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

    // Core 1: RTLS Task -> Core 1: Stm32 Send Task -> Core 1: Rasp Send Task
    // Core 0: Rasp Recv Task / Core 0: Stm32 Recv Task

    // Create Rasp Tasks
    if (xTaskCreatePinnedToCore(raspRecvTask, "Recv Task", 1 << 14, NULL, 1, &rasp_recv_task_handle, 0) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create Rasp Recv Task");
    }

    if (xTaskCreatePinnedToCore(raspSendTask, "Send Task", 1 << 14, NULL, 1, &rasp_send_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create Rasp Send Task");
    }

    // Create STM32 Tasks
    if (xTaskCreatePinnedToCore(stm32RecvTask, "Recv Task", 1 << 14, NULL, 2, &stm32_recv_task_handle, 0) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create STM32 Recv Task");
    }

    if (xTaskCreatePinnedToCore(stm32SendTask, "Send Task", 1 << 14, NULL, 2, &stm32_send_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create STM32 Send Task");
    }

    // Create RTLS Task
    if (xTaskCreatePinnedToCore(RTLSTaskWrapper, "RTLS_Task", 1 << 16, NULL, 3, &RTLS_task_handle, 1) != pdPASS) {
        ESP_LOGE("MAIN", "Failed to create RTLS Task");
    }

    ESP_LOGI("MAIN", "Setup Complete. Tasks Started.");
}

void loop()
{
}