#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "dw3000.h"
#include "DW3000_RTLS.h"
#include "rasp.h"
#include "stm32.h"

/*********************************************************************************************************************************************************
 * 														Extern Variables
 *********************************************************************************************************************************************************/

extern uint32_t last_synced_time;

extern dwt_txconfig_t txconfig_options;

extern HardwareSerial RaspHwSerial;
extern HardwareSerial Stm32HwSerial;


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
void IRAM_ATTR onRaspDataAvailable() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(rasp_recv_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

// UART ISR handler for STM32
void IRAM_ATTR onStm32DataAvailable() {
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

void setup()
{
    Serial.begin(115200);

    /***************** RTLS Setup Begin *****************/
    
    dw3000_rtls.RTLSSetup();

    Serial.println("1");

    /***************** RTLS Setup End *****************/


    /***************** Rasp Setup Begin *****************/

    RaspHwSerial.begin(RASP_UART_BAUD, SERIAL_8N1, RASP_RX_PIN, RASP_TX_PIN);
    RaspHwSerial.onReceive(onRaspDataAvailable);

    Serial.println("2");
    

    /***************** Rasp Setup End *****************/

    /***************** STM32 Setup Begin *****************/

    Stm32HwSerial.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    Stm32HwSerial.onReceive(onStm32DataAvailable);

    Serial.println("3");

    /***************** STM32 Setup End *****************/

    /***************** Semaphore Setup Begin *****************/
    
    stm32_recv_data_semaphore = xSemaphoreCreateBinary();
    rasp_recv_data_semaphore = xSemaphoreCreateBinary();
    stm32_send_flag_semaphore = xSemaphoreCreateBinary();

    xSemaphoreGive(stm32_recv_data_semaphore);
    xSemaphoreGive(rasp_recv_data_semaphore);

    /***************** Semaphore Setup End *****************/

    xthal_set_intset(0);

    Serial.println("4");

    // Core 1: RTLS Task -> Core 1: Stm32 Send Task -> Core 1: Rasp Send Task
    // Core 0: Rasp Recv Task / Core 0: Stm32 Recv Task

    // Create Rasp Tasks
    if (xTaskCreatePinnedToCore(raspRecvTask, "Recv Task", 1 << 14, NULL, 1, &rasp_recv_task_handle, 0) != pdPASS) {
        Serial.println("Failed to create Rasp Recv Task");
    }

    if (xTaskCreatePinnedToCore(raspSendTask, "Send Task", 1 << 14, NULL, 1, &rasp_send_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create Rasp Send Task");
    }

    // Create STM32 Tasks
    if (xTaskCreatePinnedToCore(stm32RecvTask, "Recv Task", 1 << 14, NULL, 2, &stm32_recv_task_handle, 0) != pdPASS) {
        Serial.println("Failed to create STM32 Recv Task");
    }

    if (xTaskCreatePinnedToCore(stm32SendTask, "Send Task", 1 << 14, NULL, 2, &stm32_send_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create STM32 Send Task");
    }

    // Create RTLS Task
    if (xTaskCreatePinnedToCore(RTLSTaskWrapper, "RTLS_Task", 1 << 16, NULL, 3, &RTLS_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create RTLS Task");
    }

    Serial.println("5");


    
    /* Create STM32 Task End */

    // vTaskStartScheduler();
}

void loop()
{
    
}
