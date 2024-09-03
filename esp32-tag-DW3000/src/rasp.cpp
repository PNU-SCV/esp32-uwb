#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rasp.h"
#include "utils.h"

/*********************************************************************************************************************************************************
 * 														Extern Variables
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
 * 														Global Variables
 *********************************************************************************************************************************************************/

HardwareSerial RaspHwSerial(2);

RaspRecvData raspRecieveData;
RaspSendData raspSendData;

float tagAngle = 0.0;
uint8_t raspCmd = 0x00;
Point2D destPoint = {3.0, 2.0};

// 수신 태스크
void raspRecvTask(void *parameter) 
{
    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("raspRecvTask");

        if (RaspHwSerial.available() >= sizeof(RaspRecvData)) 
        {
            RaspHwSerial.readBytes((char*)&raspRecieveData, sizeof(RaspRecvData));
            uint8_t receivedCRC = raspRecieveData.crc;
            uint8_t calculatedCRC = calculateCRC((uint8_t*)&raspRecieveData, sizeof(RaspRecvData) - 1);

            if (receivedCRC == calculatedCRC) 
            {
                Serial.println("Data received from Raspberry Pi");

                if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

                destPoint = {raspRecieveData.dest_x, raspRecieveData.dest_z};
                tagAngle = raspRecieveData.angle;
                raspCmd = raspRecieveData.cmd;

                xSemaphoreGive(rasp_recv_data_semaphore);

                Serial.print("RASP : ");
                Serial.print(raspCmd);
                Serial.print(", ");
                Serial.print(destPoint.x);
                Serial.print(", ");
                Serial.print(destPoint.z);
                Serial.print(", ");
                Serial.println(tagAngle);
            } 
            else 
            {
                Serial.println("CRC mismatch, data ignored");
            }
        }
    }
}


// 송신 태스크
void raspSendTask(void *parameter) 
{
    uint8_t stm32_status;

    while (true) 
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

        stm32_status = stm32Status;

        xSemaphoreGive(stm32_recv_data_semaphore);

        Serial.println("raspSendTask");

        raspSendData.stat = stm32_status;
        raspSendData.loc_x = tagPosition.x;
        raspSendData.loc_z = tagPosition.z;

        raspSendData.crc = calculateCRC((uint8_t*)&raspSendData, sizeof(RaspSendData) - 1);

        RaspHwSerial.write((const uint8_t*)&raspSendData, sizeof(RaspSendData));

        Serial.println("Data sent to Raspberry Pi");

        xTaskNotifyGive(RTLS_task_handle);
    }
}
