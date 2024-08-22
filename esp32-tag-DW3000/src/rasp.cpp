#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rasp.h"

const uint8_t raspStartByte = 0x02;
const uint8_t raspEndByte = 0x07;

HardwareSerial RaspHwSerial(2);

RaspRecvData raspRecieveData;
RaspSendData raspSendData;

extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;
extern TaskHandle_t RTLS_task_handle;

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;

extern SemaphoreHandle_t stm32_send_flag_semaphore;

extern Point2D tagPosition;

extern int8_t stm32Status;

float tagAngle = 0.0;
uint8_t raspCmd = 0x00;
Point2D destPoint = {3.0, 2.0};

// // 수신 태스크
void raspRecvTask(void *parameter) 
{
  while (true) 
  {
    // Pending for RTLS Task's notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("raspRecvTask");

    uint8_t start_byte, end_byte;

    if (RaspHwSerial.available() >= sizeof(RaspRecvData)) 
    {
      RaspHwSerial.readBytes((char*)&raspRecieveData, sizeof(RaspRecvData));

      Serial.println("Data received from Raspberry Pi");

      if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

      /* Setting target_angle, target_loc */
      start_byte = raspRecieveData.start_byte;

      destPoint = {raspRecieveData.dest_x, raspRecieveData.dest_z};
      tagAngle = raspRecieveData.angle;
      raspCmd = raspRecieveData.cmd;

      end_byte = raspRecieveData.end_byte;

      xSemaphoreGive(rasp_recv_data_semaphore);

      if(start_byte != raspStartByte || end_byte != raspEndByte) continue;

      Serial.print("RASP : ");
      Serial.print(raspCmd);
      Serial.print(", ");
      Serial.print(destPoint.x);
      Serial.print(", ");
      Serial.print(destPoint.z);
      Serial.print(", ");
      Serial.println(tagAngle);
    }
  }
}

// 송신 태스크
void raspSendTask(void *parameter) 
{
  uint8_t stm32_status;

  while (true) 
  {
    // Task pending (RTLS_Task에서 이 태스크를 트리거할 때까지 대기)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    stm32_status = stm32Status;

    xSemaphoreGive(stm32_recv_data_semaphore);

    Serial.println("raspSendTask");

    raspSendData.start_byte = raspStartByte;

    raspSendData.stat = stm32_status;

    raspSendData.loc_x = tagPosition.x;
    raspSendData.loc_z = tagPosition.z;
    
    raspSendData.end_byte = raspEndByte;

    Serial.print("Sending data: stat=");
    Serial.print(raspSendData.stat);
    Serial.print(", loc_x=");
    Serial.print(raspSendData.loc_x);
    Serial.print(", loc_z=");
    Serial.println(raspSendData.loc_z);

    // 데이터 송신
    RaspHwSerial.write((const uint8_t*)&raspSendData, sizeof(RaspSendData));

    Serial.println("Data sent to Raspberry Pi");

    // Posting to stm32 Recv Task
    xTaskNotifyGive(RTLS_task_handle);
  }
}
