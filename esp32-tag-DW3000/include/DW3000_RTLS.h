#ifndef DW3000_RTLS_H
#define DW3000_RTLS_H

#include <algorithm>
#include <queue>
#include <vector>
#include "dw3000.h"
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

#define FRAME_TYPE 0x41
#define FRAME_VERSION 0x88

#define DIST_UPDATE_RATE 0.5

#define ANCHOR_COUNT 4

#define LOC_UPDATE_RATE 0.3


struct TWR_t{
    uint8_t *tx_poll_msg;
    uint8_t *rx_resp_msg;
    double *distance;
    const Point3D *anchor_loc;
    bool is_updated;

     TWR_t& operator=(const TWR_t& other) {
        if (this == &other)
            return *this; 

        tx_poll_msg = other.tx_poll_msg;

        rx_resp_msg = other.rx_resp_msg;

        distance = other.distance;

        anchor_loc = other.anchor_loc; 

        is_updated = other.is_updated;

        return *this;
    }
};



class DW3000_RTLS {
private:
    dwt_config_t config = {
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

    uint8_t frame_seq_nb = 0;
    uint8_t rx_buffer[20];
    uint32_t status_reg = 0;
    double tof;

    Point2D tag_position;
    uint32_t last_synced_time = 0;

public:
    uint32_t getCurrentTime() {
        return millis() - last_synced_time;
    }

    Point2D getTagPosition() {
        return tag_position;
    }

    void RTLSSetup();

    bool pollAndReceive(uint8_t* poll_msg, uint8_t* resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double* distance);

    void calculateDistance(uint8_t* buffer, double* distance);
    
    void calculatePosition(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2);

    void setLocation();

    Point2D getLocation();

    virtual void RTLSTaskPrologue();

    virtual void RTLSTaskEpilogue();
};

#endif
