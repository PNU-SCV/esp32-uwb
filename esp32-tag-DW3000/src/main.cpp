#include <HardwareSerial.h>
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "dw3000.h"
#include "rtls.h"
#include "rasp.h"
#include "stm32.h"

extern uint32_t last_synced_time;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                  /* Channel number. */
    DWT_PLEN_128,       /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    1,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (128 + 1 + 8 - 8),  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,   /* STS Off */
    DWT_STS_LEN_128,    /* Ignore value when STS is disabled */
    DWT_PDOA_M0         /* PDOA mode off */
};

extern dwt_txconfig_t txconfig_options;

extern HardwareSerial RaspHwSerial;
extern HardwareSerial Stm32HwSerial;

TaskHandle_t RTLS_task_handle = NULL;

TaskHandle_t rasp_recv_task_handle = NULL;
TaskHandle_t rasp_send_task_handle = NULL;

TaskHandle_t stm32_recv_task_handle = NULL;
TaskHandle_t stm32_send_task_handle = NULL;

void setup()
{
    Serial.begin(115200);

    /***************** RTLS Setup Begin *****************/
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    
    delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    {
        while (1)
            ;
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        while (1)
            ;
    }

    // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure DW IC. See NOTE 6 below. */
    if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
        while (1)
            ;
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    // 이걸 반대편에도 추가해 보던가 하면 될듯.

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    // dwt_setcallbacks(&Tx_Callback_ISR, &Rx_Callback_ISR, &Rx_Timeout_ISR, &Rx_Error_ISR, &Spi_Error_ISR, &Spi_Ready_ISR);
    // dwt_setcallbacks(NULL, &Rx_Callback_ISR, NULL, NULL, NULL, NULL);

    // dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT);
    // 이후 SPI 설정에서 이를 덮어쓸 수도 있음.

    /***************** RTLS Setup End *****************/


    /***************** Rasp Setup Begin *****************/

    RaspHwSerial.begin(RASP_UART_BAUD, SERIAL_8N1, RASP_RX_PIN, RASP_TX_PIN);

    /***************** Rasp Setup End *****************/

    /***************** STM32 Setup Begin *****************/

    Stm32HwSerial.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

    /***************** STM32 Setup End *****************/

    // xthal_set_intset(1);

    // Core 1: RTLS Task -> Core 1: Rasp Recv Task -> Core 1: STM32 Send Task
    // Core 0: Rasp Send Task -> Core 0: STM32 Recv Task


    // Create RTLS Task
    if (xTaskCreatePinnedToCore(RTLS_Task, "RTLS_Task", 1 << 16, NULL, 3, &RTLS_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create RTLS Task");
    }

    // Create Rasp Tasks
    if (xTaskCreatePinnedToCore(raspRecvTask, "Recv Task", 1 << 10, NULL, 1, &rasp_recv_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create Rasp Recv Task");
    }

    if (xTaskCreatePinnedToCore(raspSendTask, "Send Task", 1 << 10, NULL, 1, &rasp_send_task_handle, 0) != pdPASS) {
        Serial.println("Failed to create Rasp Send Task");
    }

    // Create STM32 Tasks
    if (xTaskCreatePinnedToCore(stm32RecvTask, "Recv Task", 1 << 10, NULL, 2, &stm32_recv_task_handle, 0) != pdPASS) {
        Serial.println("Failed to create STM32 Recv Task");
    }

    if (xTaskCreatePinnedToCore(stm32SendTask, "Send Task", 1 << 10, NULL, 2, &stm32_send_task_handle, 1) != pdPASS) {
        Serial.println("Failed to create STM32 Send Task");
    }


    
    /* Create STM32 Task End */

    // vTaskStartScheduler();
}

void loop()
{
    
}
