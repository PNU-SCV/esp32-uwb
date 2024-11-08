#include "DW3000_RTLS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <algorithm>
#include <cmath>

/*********************************************************************************************************************************************************
 *                                                      Extern Variables
 *********************************************************************************************************************************************************/

extern dwt_txconfig_t txconfig_options;

extern TaskHandle_t rasp_recv_task_handle;
extern TaskHandle_t rasp_send_task_handle;
extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;

/*********************************************************************************************************************************************************
 *                                                      Global Variables
 *********************************************************************************************************************************************************/

double distance_A, distance_B, distance_C, distance_D;
double INF_distance = 1024.0;

Point3D anchor_A = {0.0, 2.0, -0.4};
Point3D anchor_B = {4.8, 2.0, -0.4};
Point3D anchor_C = {0.0, 2.0, 2.8};
Point3D anchor_D = {4.8, 2.0, 2.8};

uint8_t tx_poll_msg_A[12] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'R', 'A', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_B[12] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'R', 'B', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_C[12] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'R', 'C', 'T', 'A', 0xE0, 0, 0};
uint8_t tx_poll_msg_D[12] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'R', 'D', 'T', 'A', 0xE0, 0, 0};

uint8_t rx_resp_msg_A[20] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_B[20] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_C[20] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'C', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t rx_resp_msg_D[20] = {FRAME_TYPE, FRAME_VERSION, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'D', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

TWR_t twr[ANCHOR_COUNT + 1] = {
    {tx_poll_msg_A, rx_resp_msg_A, &distance_A, &anchor_A, false},
    {tx_poll_msg_B, rx_resp_msg_B, &distance_B, &anchor_B, false},
    {tx_poll_msg_C, rx_resp_msg_C, &distance_C, &anchor_C, false},
    {tx_poll_msg_D, rx_resp_msg_D, &distance_D, &anchor_D, false},
    {NULL, NULL, &INF_distance, NULL, false}
};

/*********************************************************************************************************************************************************
 *                                                          Constructors
 *********************************************************************************************************************************************************/

void DW3000_RTLS::RTLSSetup() {
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);

    // Delay for 2 milliseconds
    vTaskDelay(pdMS_TO_TICKS(2));

    while (!dwt_checkidlerc()) 
    {
        // Idle loop; consider adding a timeout for robustness
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        ESP_LOGE("DW3000_RTLS", "DWT initialization failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) 
    {
        ESP_LOGE("DW3000_RTLS", "DWT configuration failed");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Sort the anchors by their location */
    std::sort(twr, twr + ANCHOR_COUNT, [](const TWR_t& a, const TWR_t& b) {
        if (!a.anchor_loc || !b.anchor_loc) return false;
        if (a.anchor_loc->z != b.anchor_loc->z)
            return a.anchor_loc->z < b.anchor_loc->z;
        return a.anchor_loc->x < b.anchor_loc->x;
    });
}

/*********************************************************************************************************************************************************
 *                                                          Public Methods
 *********************************************************************************************************************************************************/

bool DW3000_RTLS::pollAndReceive(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance) {
    bool is_updated = false;

    poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(poll_msg_size, poll_msg, 0); // Zero offset in TX buffer.
    dwt_writetxfctrl(poll_msg_size, 0, 1);       // Zero offset in TX buffer, ranging.

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // Wait for transmission and reception events
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        // Optional delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(1));
    }

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
    *distance = *distance + dist_diff * LOC_UPDATE_RATE;
}

void DW3000_RTLS::calculatePosition(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2) {
    if (distance_1 < 0 || distance_2 < 0) return;

    if(anchor_1.x > anchor_2.x) {
        std::swap(anchor_1, anchor_2);
        std::swap(distance_1, distance_2);
    }

    float height_diff_1 = fabs(anchor_1.y); 
    float height_diff_2 = fabs(anchor_2.y); 

    if (distance_1 < height_diff_1 || distance_2 < height_diff_2) return;

    float horizontal_distance_1 = sqrt(pow(distance_1, 2) - pow(height_diff_1, 2));
    float horizontal_distance_2 = sqrt(pow(distance_2, 2) - pow(height_diff_2, 2));

    float dx = (pow((anchor_2.x - anchor_1.x), 2) + pow(horizontal_distance_1, 2) - pow(horizontal_distance_2, 2)) / (2 * (anchor_2.x - anchor_1.x));
    float dz = sqrt(pow(horizontal_distance_1, 2) - pow(dx, 2));

    if(std::isnan(dz)) return;

    float x = anchor_1.x + dx;
    float z = anchor_1.z + dz;

    tag_position = {x, z};

    ESP_LOGI("DW3000_RTLS", "Calculated Position - X: %.2f, Z: %.2f", x, z);
}

void DW3000_RTLS::setLocation() {
    int min_1_idx = ANCHOR_COUNT, min_2_idx = ANCHOR_COUNT;

    for (int i = 0; i < ANCHOR_COUNT; ++i) {
        if(twr[i].anchor_loc->z + POINT_EPSILON > tag_position.z || fabs(twr[i].anchor_loc->x - tag_position.x) > DELTA_X) {
            twr[i].is_updated = false;
        } else {
            twr[i].is_updated = pollAndReceive(twr[i].tx_poll_msg, twr[i].rx_resp_msg, POLL_MSG_SIZE, RESP_MSG_SIZE, twr[i].distance);
        }
    }

    for (int i = 0; i < ANCHOR_COUNT; i++) {
        if(min_1_idx != ANCHOR_COUNT && min_2_idx != ANCHOR_COUNT && twr[i].anchor_loc->z + POINT_EPSILON > tag_position.z) break;

        if(!twr[i].is_updated) continue;

        for(int j = i + 1; j < ANCHOR_COUNT && twr[i].anchor_loc->z == twr[j].anchor_loc->z; ++j) {
            if(!twr[j].is_updated) continue;            
            
            double min_sum_distance = (min_1_idx != ANCHOR_COUNT && min_2_idx != ANCHOR_COUNT) ?
                *twr[min_1_idx].distance + *twr[min_2_idx].distance : std::numeric_limits<double>::max();
            double cur_sum_distance = *twr[j].distance + *twr[i].distance;

            if (cur_sum_distance < min_sum_distance) {
                min_sum_distance = cur_sum_distance;
                min_1_idx = i;
                min_2_idx = j;

                Point3D anchor_1 = *twr[min_1_idx].anchor_loc;
                Point3D anchor_2 = *twr[min_2_idx].anchor_loc;
                float dist_1 = static_cast<float>(*twr[min_1_idx].distance);
                float dist_2 = static_cast<float>(*twr[min_2_idx].distance);

                calculatePosition(anchor_1, anchor_2, dist_1, dist_2);
            }
        }
    }

    if (min_1_idx != ANCHOR_COUNT && min_2_idx != ANCHOR_COUNT) {
        ESP_LOGI("DW3000_RTLS", "Distances - Anchor %d: %.2f, Anchor %d: %.2f", min_1_idx, *twr[min_1_idx].distance, min_2_idx, *twr[min_2_idx].distance);
    }
}

/*********************************************************************************************************************************************************
 *                                                          Task Wrapper
 *********************************************************************************************************************************************************/

void DW3000_RTLS::RTLSTaskPrologue() {
    vTaskDelay(pdMS_TO_TICKS(100));
}

void DW3000_RTLS::RTLSTaskEpilogue() {
    xTaskNotifyGive(stm32_send_task_handle);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}
