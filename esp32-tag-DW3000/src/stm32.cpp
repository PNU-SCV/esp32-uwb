#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include <cmath>
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("stm32RecvTask");

    if (Stm32HwSerial.available() >= sizeof(STM32RecvData)) 
    {
      Stm32HwSerial.readBytes((char*)&stm32RecieveData, sizeof(STM32RecvData));

      if(xSemaphoreTake(stm32_recv_data_semaphore, portMAX_DELAY) == pdFALSE) continue;

      stm32RecieveData.status = 0;
      stm32Status = stm32RecieveData.status;

      xSemaphoreGive(stm32_recv_data_semaphore);

      Serial.print("STM32 Recv: ");
      Serial.println(stm32RecieveData.status);
    }
  }
}

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
      Serial.println("STOP");
    }
    else if (min(angle_diff, 360.0f - angle_diff) < ANGLE_EPSILON) 
    {
      stm32SendData.cmd = CMD_FORWARD; 
      Serial.println("FORWARD");
    }
    else if (angle_diff <= 180.0f)
    {
      stm32SendData.cmd = CMD_CLOCKWISE_ROTATE;
      Serial.println("CLOCKWISE ROTATE");
    }
    else 
    {
      stm32SendData.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
      Serial.println("COUNTERCLOCKWISE ROTATE");
    }


    //if(xSemaphoreTake(stm32_send_flag_semaphore, 0) == pdFALSE) continue;

    Serial.println("stm32SendTask");

    // stm32_send_data.cmd = CMD_COUNTERCLOCKWISE_ROTATE;
    // if(tag_position.z < 1) stm32_send_data.cmd = CMD_FORWARD;
    // else stm32_send_data.cmd = CMD_STOP;

    Stm32HwSerial.write((const uint8_t*)&stm32SendData, sizeof(STM32SendData));

    xTaskNotifyGive(rasp_send_task_handle);
  }
}
