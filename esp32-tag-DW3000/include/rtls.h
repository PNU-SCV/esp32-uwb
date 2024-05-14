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

#define TIME_SLOT_SEQ_LENTH 250
#define TIME_SLOT_LENGTH 60
#define TIME_SLOT_COUNT 4
#define ANCHOR_COUNT 2

// TODO: Change this value to the desired time slot index
#define TIME_SLOT_IDX 0

void RTLS_Task(void *parameter);

#endif