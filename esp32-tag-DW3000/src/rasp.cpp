#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rasp.h"

HardwareSerial RaspHwSerial(1);

volatile RaspRecvData rasp_recieve_data;
volatile RaspSendData rasp_send_data;

extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;
extern TaskHandle_t RTLS_task_handle;

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;

extern Point2D tag_position;
extern Point2D target_loc;
extern float tag_angle;

uint8_t rasp_cmd = 0;

uint8_t rasp_stat = 0;

// // 수신 태스크
void raspRecvTask(void *parameter) 
{
  while (true) 
  {
    // Pending for RTLS Task's notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("raspRecvTask");

    if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    if (RaspHwSerial.available() >= sizeof(RaspRecvData)) 
    {
      RaspHwSerial.readBytes((char*)&rasp_recieve_data, sizeof(RaspRecvData));

      Serial.println("Data received from Raspberry Pi");

      /* Setting target_angle, target_loc */
      target_loc = {rasp_recieve_data.dest_x, rasp_recieve_data.dest_z};
      tag_angle = rasp_recieve_data.angle;
      rasp_cmd = rasp_recieve_data.cmd;
    }

    xSemaphoreGive(rasp_recv_data_semaphore);
  }
}

// 송신 태스크
void raspSendTask(void *parameter) 
{
  while (true) 
  {
    // Task pending (RTLS_Task에서 이 태스크를 트리거할 때까지 대기)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    Serial.println("raspSendTask");

    rasp_send_data.stat = rasp_stat;
    rasp_send_data.loc_x = tag_position.x;
    rasp_send_data.loc_z = tag_position.z;

    // 데이터 송신
    RaspHwSerial.write((const uint8_t*)&rasp_send_data, sizeof(RaspSendData));

    Serial.println("Data sent to Raspberry Pi");

    xSemaphoreGive(stm32_recv_data_semaphore);

    // Posting to stm32 Recv Task
    xTaskNotifyGive(RTLS_task_handle);
  }
}
