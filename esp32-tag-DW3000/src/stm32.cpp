#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rtls.h"
#include "rasp.h"
#include "stm32.h"
#include "utils.h"

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

HardwareSerial Stm32HwSerial(1);

STM32RecvData stm32RecieveData = {0};
STM32SendData stm32SendData = {0};


uint8_t stm32Status = 0;

/*********************************************************************************************************************************************************
 * 														    FreeRTOS Tasks
 *********************************************************************************************************************************************************/

void stm32RecvTask(void *parameter) 
{
  while (true) 
  {
    /* Task pending (Rasp Recv 에서 이 태스크를 트리거할 때까지 대기) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("stm32RecvTask");

    if (Stm32HwSerial.available() >= sizeof(STM32RecvData)) 
    {
      Stm32HwSerial.readBytes((char*)&stm32RecieveData, sizeof(STM32RecvData));

      if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

      stm32Status = stm32RecieveData.status;

      xSemaphoreGive(stm32_recv_data_semaphore);

      Serial.print("STM32 Recv: ");
      Serial.println(stm32Status);
    }
  }
}

/* 송신 태스크 */
void stm32SendTask(void *parameter) 
{
  uint8_t stm32_status, rasp_cmd;
  Point2D tag_position, dest_point;
  float tag_angle;

  while (true) 
  {
    /* Task pending (raspRecvTask에서 이 태스크를 트리거할 때까지 대기) */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    stm32_status = stm32RecieveData.status;

    xSemaphoreGive(stm32_recv_data_semaphore);

    if(xSemaphoreTake(rasp_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

    rasp_cmd = raspCmd;
    dest_point = destPoint;
    tag_angle = tagAngle;

    xSemaphoreGive(rasp_recv_data_semaphore);

    tag_position = tagPosition;

    //if(xSemaphoreTake(stm32_send_flag_semaphore, 0) == pdFALSE) continue;

    Serial.println("stm32SendTask");

    if (tag_position == dest_point || rasp_cmd == CMD_STOP || stm32_status != 0x00) 
    {
      stm32SendData.cmd = CMD_STOP;
      Serial.println("STOP");
    }
    else if (std::abs(tag_angle - getAngle(dest_point, tag_position)) < ANGLE_EPSILON) 
    {
      stm32SendData.cmd = CMD_FORWARD;
      Serial.println("FORWARD");
    }
    else if (tag_angle < getAngle(dest_point, tag_position)) 
    {
      stm32SendData.cmd = CMD_CLOCKWISE_ROTATE;
      Serial.println("CLOCK");
    }
    else 
    {
      stm32SendData.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
      Serial.println("ANTT-CLOCK");
    }

    // stm32_send_data.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
    // if(tag_position.z < 1) stm32_send_data.cmd = CMD_FORWARD;
    // else stm32_send_data.cmd = CMD_STOP;

    /* Send CMD to STM32 */
    Stm32HwSerial.write((const uint8_t*)&stm32SendData, sizeof(STM32SendData));

    /* Posting to RTLS Task */
    xTaskNotifyGive(rasp_send_task_handle);
  }
}
