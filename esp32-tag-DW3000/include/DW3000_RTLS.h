#ifndef DW3000_RTLS_H
#define DW3000_RTLS_H

#include <Arduino.h>
#include <algorithm>
#include <queue>
#include <vector>
#include "dw3000.h"
#include "rtls.h"

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

    uint8_t tx_time_sync_msg[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'S', 'Y', 'N', 0xE0, 0, 0};

    // Anchor Configuration
    Point3D anchor_A = {1.5, 0, 0};
    Point3D anchor_B = {3.0, 0, 0};

    uint8_t tx_poll_msg_A[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'T', 'A', 0xE0, 0, 0};
    uint8_t tx_poll_msg_B[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'B', 'T', 'A', 0xE0, 0, 0};

    uint8_t rx_resp_msg_A[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rx_resp_msg_B[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    uint8_t frame_seq_nb = 0;
    uint8_t rx_buffer[20];
    uint32_t status_reg = 0;
    double tof;
    double distance_A, distance_B;
    double INF_distance = 1024.0;

    TWR_t twr[3] = {
        {tx_poll_msg_A, rx_resp_msg_A, &distance_A, &anchor_A, false},
        {tx_poll_msg_B, rx_resp_msg_B, &distance_B, &anchor_B, false},
        {NULL, NULL, &INF_distance, NULL, false}
    };

    Point2D tag_position;
    uint32_t last_synced_time = 0;

public:
    uint32_t getCurrentTime() {
        return millis() - last_synced_time;
    }

    Point2D getTagPosition() {
        return tag_position;
    }

    void RTLS_Setup();

    bool poll_And_Recieve(uint8_t* poll_msg, uint8_t* resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double* distance);

    void calculate_Distance(uint8_t* buffer, double* distance);

    void broadcast_Time_Sync_Msg(uint8_t* sync_msg, uint8_t sync_msg_size);
    
    void calculate_Position(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2);

    void RTLS_Task(void* parameter);

    virtual void RTLS_Task_Prologue();

    virtual void RTLS_Task_Epilogue();
};

#endif
