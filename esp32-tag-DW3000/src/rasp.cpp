#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rasp.h"
#include "main.h"
#include "utils.h"
#include <string.h>

/*********************************************************************************************************************************************************
 *                                                      Extern Variables
 *********************************************************************************************************************************************************/

const uint8_t raspStartByte = 0x02;
const uint8_t raspEndByte = 0x07;

extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;
extern TaskHandle_t RTLS_task_handle;

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;

extern SemaphoreHandle_t stm32_send_flag_semaphore;

extern Point2D tagPosition;

extern int8_t stm32Status;

/*********************************************************************************************************************************************************
 *                                                      Global Variables
 *********************************************************************************************************************************************************/

static const int BUFFER_SIZE = 1024;

RaspRecvData raspRecieveData;
RaspSendData raspSendData;

float tagAngle = 0.0;
uint8_t raspCmd = 0x00;
Point2D destPoint = {1.4, 1.0};

// Logging tag
static const char* TAG = "RASP";

/*********************************************************************************************************************************************************
 *                                                      Tasks
 *********************************************************************************************************************************************************/

// Task for receiving data from Raspberry Pi via UART
void raspRecvTask(void *parameter) 
{
    QueueHandle_t rasp_uart_queue = (QueueHandle_t)parameter;
    uart_event_t event;
    uint8_t* buffer = (uint8_t*) malloc(BUFFER_SIZE);

    while (true) 
    {
        if (xQueueReceive(rasp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) 
        {
            if (event.type == UART_DATA && event.size >= sizeof(RaspRecvData)) 
            {
                int len = uart_read_bytes(RASP_UART_NUM, buffer, event.size, portMAX_DELAY);
                if (len >= sizeof(RaspRecvData)) 
                {
                    memcpy(&raspRecieveData, buffer, sizeof(RaspRecvData));

                    uint8_t receivedCRC = raspRecieveData.crc;
                    uint8_t calculatedCRC = calculateCRC((uint8_t*)&raspRecieveData, sizeof(RaspRecvData) - 1);

                    if (receivedCRC == calculatedCRC) 
                    {
                        ESP_LOGI(TAG, "Data received from Raspberry Pi");

                        if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

                        destPoint = {raspRecieveData.dest_x, raspRecieveData.dest_z};
                        tagAngle = raspRecieveData.angle;
                        raspCmd = raspRecieveData.cmd;

                        xSemaphoreGive(rasp_recv_data_semaphore);

                        ESP_LOGI(TAG, "RASP: Cmd=%d, X=%.2f, Z=%.2f, Angle=%.2f", 
                                raspRecieveData.cmd, raspRecieveData.dest_x, 
                                raspRecieveData.dest_z, raspRecieveData.angle);

                        // Notify stm32SendTask to process the new data
                        xTaskNotifyGive(stm32_send_task_handle);
                    } 
                    else 
                    {
                        ESP_LOGW(TAG, "CRC mismatch, data ignored");

                        // Flush the buffer in case of CRC mismatch
                        uart_flush(RASP_UART_NUM);
                    }
                }
            }
        }
    }
    free(buffer);
}

// Task for sending data to Raspberry Pi
void raspSendTask(void *parameter) 
{
    uint8_t stm32_status;

    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

        stm32_status = stm32Status;

        xSemaphoreGive(stm32_recv_data_semaphore);

        ESP_LOGI(TAG, "raspSendTask");

        raspSendData.stat = stm32_status;
        raspSendData.loc_x = tagPosition.x;
        raspSendData.loc_z = tagPosition.z;

        // Calculate and assign CRC
        raspSendData.crc = calculateCRC((uint8_t*)&raspSendData, sizeof(RaspSendData) - 1);

        // Send data to Raspberry Pi
        uart_write_bytes(RASP_UART_NUM, (const char*)&raspSendData, sizeof(RaspSendData));

        ESP_LOGI(TAG, "Data sent to Raspberry Pi: Status=%d, X=%.2f, Z=%.2f", 
                raspSendData.stat, raspSendData.loc_x, raspSendData.loc_z);

        // Notify the RTLS task
        xTaskNotifyGive(RTLS_task_handle);
    }
}
