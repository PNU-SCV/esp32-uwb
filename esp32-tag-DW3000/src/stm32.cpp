#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <cmath>
#include "rasp.h"
#include "stm32.h"
#include "utils.h"
#include <algorithm>
#include <string.h>

/*********************************************************************************************************************************************************
 * 														Extern Variables
 *********************************************************************************************************************************************************/

extern TaskHandle_t RTLS_task_handle;
extern TaskHandle_t rasp_recv_task_handle;
extern TaskHandle_t rasp_send_task_handle;

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;

extern Point2D tagPosition;
extern float tagAngle;

extern Point2D destPoint;
extern uint8_t raspCmd;

/*********************************************************************************************************************************************************
 * 														Global Variables
 *********************************************************************************************************************************************************/

static const int STM32_UART_NUM = UART_NUM_1;  // UART number for STM32 communication
static const int BUFFER_SIZE = 1024;

STM32RecvData stm32RecieveData = {0};
STM32SendData stm32SendData = {0};

uint8_t stm32Status = 0;

// Logging tag
static const char* TAG = "STM32";

/*********************************************************************************************************************************************************
 * 														    FreeRTOS Tasks
 *********************************************************************************************************************************************************/

// ISR Callback Function: Recv from STM32 via UART
void stm32RecvTask(void *parameter) 
{
    uint8_t buffer[BUFFER_SIZE];

    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "stm32RecvTask");

        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(STM32_UART_NUM, (size_t*)&length));

        if (length >= sizeof(STM32RecvData)) 
        {
            uart_read_bytes(STM32_UART_NUM, buffer, sizeof(STM32RecvData), portMAX_DELAY);
            memcpy(&stm32RecieveData, buffer, sizeof(STM32RecvData));

            if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

            stm32RecieveData.status = 0;
            stm32Status = stm32RecieveData.status;

            xSemaphoreGive(stm32_recv_data_semaphore);

            ESP_LOGI(TAG, "STM32 Recv: Status=%d", stm32RecieveData.status);
        }
    }
}

// FreeRTOS Task
// Pending for: RTLSTask, Posting for raspSendTask
void stm32SendTask(void *parameter) 
{
    uint8_t stm32_status, rasp_cmd;
    Point2D tag_position, dest_point;
    float tag_angle;
    float angle_diff;

    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

        stm32_status = stm32RecieveData.status;
        stm32_status = 0;

        xSemaphoreGive(stm32_recv_data_semaphore);

        if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

        rasp_cmd = raspCmd;
        dest_point = destPoint;
        tag_angle = tagAngle;

        xSemaphoreGive(rasp_recv_data_semaphore);

        tag_position = tagPosition;

        angle_diff = getAngleDiff(getAngle(dest_point, tag_position), tag_angle);

        if(std::isnan(tag_position.x) || std::isnan(tag_position.z) || tag_position == dest_point || rasp_cmd == CMD_STOP || stm32_status != 0x00) 
        {    
            stm32SendData.cmd = CMD_STOP;
            ESP_LOGI(TAG, "STOP");
        }
        else if (std::min(angle_diff, 360.0f - angle_diff) < ANGLE_EPSILON) 
        {
            stm32SendData.cmd = CMD_FORWARD;
            ESP_LOGI(TAG, "FORWARD");
        }
        else if (angle_diff <= 180.0f)
        {
            stm32SendData.cmd = CMD_CLOCKWISE_ROTATE;
            ESP_LOGI(TAG, "CLOCKWISE ROTATE");
        }
        else 
        {
            stm32SendData.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
            ESP_LOGI(TAG, "COUNTERCLOCKWISE ROTATE");
        }

        ESP_LOGI(TAG, "stm32SendTask");

        uart_write_bytes(STM32_UART_NUM, (const char*)&stm32SendData, sizeof(STM32SendData));

        xTaskNotifyGive(rasp_send_task_handle);
    }
}