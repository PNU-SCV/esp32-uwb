#include "DW3000_RTLS.h"

/*********************************************************************************************************************************************************
 * 														Extern Variables
 *********************************************************************************************************************************************************/

extern dwt_txconfig_t txconfig_options;

extern TaskHandle_t rasp_recv_task_handle;
extern TaskHandle_t rasp_send_task_handle;
extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;

/*********************************************************************************************************************************************************
 * 														Global Variables
 *********************************************************************************************************************************************************/

double distance_A, distance_B, distance_C, distance_D;
double INF_distance = 1024.0;

Point3D anchor_A = {1.5, 0.0, 0.0};
Point3D anchor_B = {3.0, 0.0, 0.0};
Point3D anchor_C = {0.0, 0.0, 0.0};
Point3D anchor_D = {4.5, 0.0, 0.0};

uint8_t tx_poll_msg_A[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_B[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'B', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_C[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'C', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_D[12] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'D', 'T', 'A', 0xE0, 0, 0};

uint8_t rx_resp_msg_A[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_B[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_C[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'C', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_D[20] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'D', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

TWR_t twr[ANCHOR_COUNT + 1] = {
    {tx_poll_msg_A, rx_resp_msg_A, &distance_A, &anchor_A, false},
    {tx_poll_msg_B, rx_resp_msg_B, &distance_B, &anchor_B, false},
    {tx_poll_msg_C, rx_resp_msg_C, &distance_C, &anchor_C, false},
    {tx_poll_msg_D, rx_resp_msg_D, &distance_D, &anchor_D, false},
    {NULL, NULL, &INF_distance, NULL, false}
};

/*********************************************************************************************************************************************************
 * 														    Constructors
 *********************************************************************************************************************************************************/

void DW3000_RTLS::RTLSSetup() {
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    
    delay(2);

    while (!dwt_checkidlerc()) 
    {
        while (1)
            ;
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        while (1)
            ;
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) 
    {
        while (1)
            ;
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Sort the anchors by their location */
    std::sort(twr, twr + ANCHOR_COUNT, [&](TWR_t a, TWR_t b) { 
        return a.anchor_loc->z == b.anchor_loc->z ? a.anchor_loc->x < b.anchor_loc->x : a.anchor_loc->z < b.anchor_loc->z;
    });
}

/*********************************************************************************************************************************************************
 * 														    Public Methods
 *********************************************************************************************************************************************************/

bool DW3000_RTLS::pollAndRecieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance) {
    bool is_updated = false;

    poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(poll_msg_size, poll_msg, 0); // Zero offset in TX buffer.
    dwt_writetxfctrl(poll_msg_size, 0, 1);       // Zero offset in TX buffer, ranging.

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // vTaskDelay(pdMS_TO_TICKS(1));

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    frame_seq_nb = (frame_seq_nb + 1) % 256;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                calculateDistance(rx_buffer, distance);
                is_updated = true;
            }
        }
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    return is_updated;
}

void DW3000_RTLS::calculateDistance(uint8_t* buffer, double *distance) {
    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    int32_t rtd_init, rtd_resp;
    float clockOffsetRatio;

    poll_tx_ts = dwt_readtxtimestamplo32();
    resp_rx_ts = dwt_readrxtimestamplo32();
    resp_msg_get_ts(&buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    resp_msg_get_ts(&buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    rtd_init = resp_rx_ts - poll_tx_ts;
    rtd_resp = resp_tx_ts - poll_rx_ts;

    double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
    double dist_diff = tof * SPEED_OF_LIGHT - *distance;
    *distance = *distance + dist_diff * 0.1;
}

void DW3000_RTLS::calculatePosition(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2) {
    if(anchor_1.x > anchor_2.x) {
        std::swap(anchor_1, anchor_2);
        std::swap(distance_1, distance_2);
    }

    float dx = (pow((anchor_2.x - anchor_1.x), 2) + pow(distance_1, 2) - pow(distance_2, 2)) / (2 * (anchor_2.x - anchor_1.x));
    float dz = sqrt(pow(distance_1, 2) - pow(dx, 2));

    float x = anchor_1.x + dx;
    float z = anchor_1.z + dz;

    tag_position = {x, z};

    Serial.print("X : ");
    Serial.print(x);
    Serial.print(", Z : ");
    Serial.println(z);
}

void DW3000_RTLS::setLocation() {
    int min_1_idx, min_2_idx;

    for (int i = 0; i < ANCHOR_COUNT; ++i) {
        twr[i].is_updated = pollAndRecieve(twr[i].tx_poll_msg, twr[i].rx_resp_msg, POLL_MSG_SIZE, RESP_MSG_SIZE, twr[i].distance);
    }

    min_1_idx = min_2_idx = ANCHOR_COUNT;

    for (int i = 1; i < ANCHOR_COUNT; i++) {
        if (!twr[i - 1].is_updated || !twr[i].is_updated || twr[i - 1].anchor_loc->z != twr[i].anchor_loc->z) continue;

        double min_sum_distance = *twr[min_1_idx].distance + *twr[min_2_idx].distance;
        double cur_sum_distance = *twr[i - 1].distance + *twr[i].distance;

        if (cur_sum_distance < min_sum_distance) {
            min_sum_distance = cur_sum_distance;
            min_1_idx = i - 1;
            min_2_idx = i;
        }
    }

    if (min_1_idx != ANCHOR_COUNT && min_2_idx != ANCHOR_COUNT) {
        Serial.print(min_1_idx);
        Serial.print(": ");
        Serial.println(*twr[min_1_idx].distance);

        Serial.print(min_2_idx);
        Serial.print(": ");
        Serial.println(*twr[min_2_idx].distance);

        Point3D anchor_1 = *twr[min_1_idx].anchor_loc, anchor_2 = *twr[min_2_idx].anchor_loc;
        float dist_1 = (float)*twr[min_1_idx].distance, dist_2 = (float)*twr[min_2_idx].distance;

        calculatePosition(anchor_1, anchor_2, dist_1, dist_2);
    }
}

void DW3000_RTLS::RTLSTaskPrologue() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void DW3000_RTLS::RTLSTaskEpilogue() {
    xTaskNotifyGive(stm32_send_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

Point2D DW3000_RTLS::getLocation() {
    return tag_position;
}
