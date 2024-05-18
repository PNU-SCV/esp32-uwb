#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "rasp.h"

HardwareSerial hwSerial(1);

volatile RaspRecvData rasp_recieve_data;
volatile RaspSendData rasp_send_data;

// // 수신 태스크
void raspRecvTask(void *parameter) {
  while (true) {
    if (hwSerial.available() >= sizeof(RaspRecvData)) {
      hwSerial.readBytes((char*)&rasp_recieve_data, sizeof(RaspRecvData));

      // 여기에서 수신 데이터 처리
      
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // 데이터 폴링 빈도 조절
  }
}

// 송신 태스크
void raspSendTask(void *parameter) {
  while (true) {
    // Task pending (RTLS_Task에서 이 태스크를 트리거할 때까지 대기)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 데이터 송신
    hwSerial.write((const uint8_t*)&rasp_send_data, sizeof(RaspSendData));

    Serial.println("Data sent to Raspberry Pi");
  }
}
