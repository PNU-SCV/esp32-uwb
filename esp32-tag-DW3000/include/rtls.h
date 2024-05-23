#ifndef __RTLS_H
#define __RTLS_H

#include <cstdint>
#include <cstring>
#include "point.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 1000
/* No obstacle : 16385, PC: 16372 */
#define TX_ANT_DLY 16372// 16385
#define RX_ANT_DLY 16372 // 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400
#define POLL_MSG_SIZE (uint8_t) 12
#define RESP_MSG_SIZE (uint8_t) 20

#define DIST_UPDATE_RATE 0.5

#define FRAME_CYCLE_TIME 500
#define TIME_SLOT_LENGTH 100
#define TIME_SLOT_COUNT 2
#define TIME_SLOT_INX 0

#define ANCHOR_COUNT 2

struct TWR_t{
    uint8_t *tx_poll_msg;
    uint8_t *rx_resp_msg;
    double *distance;
    const Point3D *anchor_loc;
    bool is_updated;

     TWR_t& operator=(const TWR_t& other) {
        if (this == &other)
            return *this; 

        if (other.tx_poll_msg) {
            tx_poll_msg = new uint8_t[sizeof(other.tx_poll_msg)];
            std::memcpy(tx_poll_msg, other.tx_poll_msg, sizeof(other.tx_poll_msg));
        } else {
            tx_poll_msg = nullptr;
        }

        if (other.rx_resp_msg) {
            rx_resp_msg = new uint8_t[sizeof(other.rx_resp_msg)];
            std::memcpy(rx_resp_msg, other.rx_resp_msg, sizeof(other.rx_resp_msg));
        } else {
            rx_resp_msg = nullptr;
        }

        if (other.distance) {
            distance = new double;
            *distance = *(other.distance);
        } else {
            distance = nullptr;
        }

        anchor_loc = other.anchor_loc; 
        is_updated = other.is_updated;

        return *this;
    }
};

void RTLS_Task(void *parameter);

#endif
