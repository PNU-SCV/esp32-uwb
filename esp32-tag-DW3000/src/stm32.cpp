#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rtls.h"
#include "rasp.h"
#include "stm32.h"

extern TaskHandle_t RTLS_task_handle;
extern TaskHandle_t rasp_recv_task_handle;
extern TaskHandle_t rasp_send_task_handle;

extern uint8_t rasp_cmd;
extern uint8_t rasp_stat;

HardwareSerial Stm32HwSerial(1);

STM32RecvData stm32_recieve_data;
STM32SendData stm32_send_data;

extern SemaphoreHandle_t stm32_recv_data_semaphore;
extern SemaphoreHandle_t rasp_recv_data_semaphore;

extern SemaphoreHandle_t stm32_send_flag_semaphore;

extern Point2D target_loc;
extern Point2D tag_position;
extern float tag_angle;

float getAngle(Point2D target, Point2D cur);

void stm32RecvTask(void *parameter) 
{
  while (true) 
  {
    /* Task pending (Rasp Recv 에서 이 태스크를 트리거할 때까지 대기) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    Serial.println("stm32RecvTask");

    if (Stm32HwSerial.available() >= sizeof(STM32RecvData)) 
    {
      Stm32HwSerial.readBytes((char*)&stm32_recieve_data, sizeof(STM32RecvData));

      rasp_stat = stm32_recieve_data.status;

      Serial.print("STM32 Recv: ");
      Serial.println(rasp_stat);
    }

    xSemaphoreGive(stm32_recv_data_semaphore);
  }
}

/* 송신 태스크 */
void stm32SendTask(void *parameter) 
{
  while (true) 
  {
    /* Task pending (raspRecvTask에서 이 태스크를 트리거할 때까지 대기) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) 
    {
      xSemaphoreGive(stm32_recv_data_semaphore);
      continue;
    }

    //if(xSemaphoreTake(stm32_send_flag_semaphore, 0) == pdFALSE) continue;

    Serial.println("stm32SendTask");

    if (tag_position == target_loc || rasp_cmd == CMD_STOP || stm32_recieve_data.status == 0x01) 
    {
      stm32_send_data.cmd = CMD_STOP;
    }
    else if (std::abs(tag_angle - getAngle(target_loc, tag_position)) < ANGLE_EPSILON) 
    {
      stm32_send_data.cmd = CMD_FORWARD;
    }
    else if (tag_angle < getAngle(target_loc, tag_position)) 
    {
      stm32_send_data.cmd = CMD_CLOCKWISE_ROTATE;
    }
    else 
    {
      stm32_send_data.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
    }

    // stm32_send_data.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
    if(tag_position.z < 1) stm32_send_data.cmd = CMD_FORWARD;
    else stm32_send_data.cmd = CMD_STOP;

    /* Send CMD to STM32 */
    Stm32HwSerial.write((const uint8_t*)&stm32_send_data, sizeof(STM32SendData));

    xSemaphoreGive(stm32_recv_data_semaphore);

    xSemaphoreGive(rasp_recv_data_semaphore);

    /* Posting to RTLS Task */
    xTaskNotifyGive(rasp_send_task_handle);
  }
}

float getAngle(Point2D target, Point2D cur) 
{
  float dx = target.x - cur.x;
  float dz = target.z - cur.z;

  return std::atan2(dz, dx) * 180 / PI;
}
