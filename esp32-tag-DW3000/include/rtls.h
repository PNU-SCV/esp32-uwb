#ifndef __RTLS_H
#define __RTLS_H

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400
#define POLL_MSG_SIZE 12
#define RESP_MSG_SIZE 20

#define Rx_Delay 1000

// void Tx_Task(void *parameter);

// void Rx_Task(void *parameter);

// void Rx_Timeout_Task(void *parameter);

// void Rx_Error_Task(void *parameter);

// void Spi_Error_Task(void *parameter);

// void Spi_Ready_Task(void *parameter);

void RTLS_Task(void *parameter);

#endif