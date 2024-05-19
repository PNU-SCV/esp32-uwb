#include <Arduino.h>
#include <algorithm>
#include <queue>
#include <vector>
#include "dw3000.h"
#include "rtls.h"

extern TaskHandle_t rasp_recv_task_handle;
extern TaskHandle_t rasp_send_task_handle;
extern TaskHandle_t stm32_send_task_handle;
extern TaskHandle_t stm32_recv_task_handle;

static uint8_t tx_time_sync_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'S', 'Y', 'N', 0xE0, 0, 0};

uint32_t last_synced_time = 0;

/***************************** Anchor Configuration Begin *****************************/

                          /* x,   y,   z */
const Point3D anchor_A = { 1.5,   0,   0};
const Point3D anchor_B = { 3.0,   0,   0};

static uint8_t tx_poll_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'A', 'T', 'A', 0xE0, 0, 0};
static uint8_t tx_poll_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'B', 'T', 'A', 0xE0, 0, 0};

static uint8_t rx_resp_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'R', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance_A, distance_B;

static double INF_distance = 1024.0;

/***************************** Anchor Configuration End *****************************/

static TWR_t twr[] = {  
                        {tx_poll_msg_A, rx_resp_msg_A, &distance_A, &anchor_A},
                        {tx_poll_msg_B, rx_resp_msg_B, &distance_B, &anchor_B},

                        /* For INF */
                        {NULL         , NULL         , &INF_distance, NULL}
};

extern Point2D tag_position;

uint32_t getCurrentTime(void)
{
    return millis() - last_synced_time;
}

bool poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance);

void calculate_Distance(uint8_t* buffer, double *distance);

void broadcast_Time_Sync_Msg(uint8_t *sync_msg, uint8_t sync_msg_size);

void calculate_Position(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2);

void RTLS_Task(void *parameter)
{
    int min_1_idx, min_2_idx;

    Serial.println("Range RX");
    Serial.println("Setup over........");

    while (1)
    {
        /* Broadcast time sync message */
        broadcast_Time_Sync_Msg(tx_time_sync_msg, sizeof(tx_time_sync_msg));
        Serial.println("Time Synced!");

        vTaskDelay(pdMS_TO_TICKS(2));

        min_1_idx = min_2_idx = ANCHOR_COUNT;

        for(int i = 0; i < ANCHOR_COUNT; ++i) 
        {
            bool is_updated = poll_And_Recieve(twr[i].tx_poll_msg, twr[i].rx_resp_msg, POLL_MSG_SIZE, RESP_MSG_SIZE, twr[i].distance);

            if(is_updated == false) continue;

            double min_1_dist = *twr[min_1_idx].distance, min_2_dist = *twr[min_2_idx].distance;
            double cur_dist = *twr[i].distance;

            if(cur_dist < min_2_dist) 
            {
                min_2_dist = cur_dist;
                min_2_idx = i;
            }

            if(min_2_dist < min_1_dist)
            {
                std::swap(min_1_idx, min_2_idx);
            }
        }

        if(min_1_idx != ANCHOR_COUNT && min_2_idx != ANCHOR_COUNT) 
        {
            Point3D anchor_1 = *twr[min_1_idx].anchor_loc, anchor_2 = *twr[min_2_idx].anchor_loc;
            float dist_1 = (float)*twr[min_1_idx].distance, dist_2 = (float)*twr[min_2_idx].distance;

            /* Calulate current position rest of time slots (and, post call end of the function)*/
            calculate_Position(anchor_1, anchor_2, dist_1, dist_2);
        }

        // Posting to raspRecvTask
        xTaskNotifyGive(rasp_recv_task_handle);

        // Pending for stm32SendTask
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(FRAME_CYCLE_TIME - getCurrentTime()));
    }
}


bool poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance)
{
    bool is_updated = false;

    /* Polling for TDMA TIME SLOT */
    //while(((getCurrentTime() % TIME_SLOT_SEQ_LENTH) / TIME_SLOT_LENGTH) != time_slot_idx);

    /* Delay for TDMA Time Slot */
    // vTaskDelay(pdMS_TO_TICKS((FRAME_CYCLE_TIME + time_slot_idx * TIME_SLOT_LENGTH - (getCurrentTime() % FRAME_CYCLE_TIME)) % FRAME_CYCLE_TIME));
    
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(poll_msg_size, poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(poll_msg_size, 0, 1);       /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));
    
    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;
    
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;
    
        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    
        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        
            /* Check that the frame is the expected response from the companion "SS TWR responder" example.
            * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                calculate_Distance(rx_buffer, distance);
                is_updated = true;
            }
        }
    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    return is_updated;           
}

void calculate_Distance(uint8_t* buffer, double *distance)
{
    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
    int32_t rtd_init, rtd_resp;
    float clockOffsetRatio;

    // 타임스탬프 추출
    poll_tx_ts = dwt_readtxtimestamplo32();       // 폴 메시지의 전송 타임스탬프
    resp_rx_ts = dwt_readrxtimestamplo32();       // 응답 메시지의 수신 타임스탬프
    resp_msg_get_ts(&buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);  // 폴 메시지의 수신 타임스탬프
    resp_msg_get_ts(&buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);  // 응답 메시지의 전송 타임스탬프

    // 캐리어 적분기 값 읽기 및 클럭 오프셋 비율 계산
    clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

    // 왕복 지연 시간(Round Trip Delay, RTD) 계산
    rtd_init = resp_rx_ts - poll_tx_ts;           // 초기 왕복 지연 시간
    rtd_resp = resp_tx_ts - poll_rx_ts;           // 응답 왕복 지연 시간

    // 시간 오차를 고려한 실제 비행 시간(Time of Flight, TOF) 계산
    double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;

    double dist_diff = tof * SPEED_OF_LIGHT - *distance;
    
    // 거리 계산 (TOF × 빛의 속도)
    /* Moving Average */ 
    // *distance = *distance + dist_diff * DIST_UPDATE_RATE;'

    /* Moving Average with log */
    *distance = *distance + (dist_diff < 0 ? -1 : 1) * log10(abs(dist_diff) + 1);
    
}

void calculate_Position(Point3D anchor_1, Point3D anchor_2, float distance_1, float distance_2) {
    float anchor_x_diff;
    float dx, dz;
    float x, z;

    if(anchor_1.x > anchor_2.x) {
        Point3D temp = anchor_1;
        anchor_1 = anchor_2;
        anchor_2 = temp;
        
        std::swap(distance_1, distance_2);
    }

    anchor_x_diff = anchor_2.x - anchor_1.x;

    dx = (pow(anchor_x_diff, 2.0) + pow(distance_1, 2.0) - pow(distance_2, 2.0)) / (2.0 * anchor_x_diff);

    if(distance_1 < dx) return;

    dz = pow(pow(distance_1, 2.0) - pow(dx, 2.0), 0.5);

    x = anchor_1.x + dx;
    z = anchor_1.z + dz;

    tag_position = {x, z};

    Serial.print("Tag Position: x = ");
    Serial.print(x, 2);
    Serial.print(", z = ");
    Serial.println(z, 2);
}


void broadcast_Time_Sync_Msg(uint8_t *sync_msg, uint8_t sync_msg_size)
{
    /* Delay for TDMA time slot */
    // vTaskDelay(pdMS_TO_TICKS((FRAME_CYCLE_TIME + time_slot_idx * TIME_SLOT_LENGTH - (getCurrentTime() % FRAME_CYCLE_TIME)) % FRAME_CYCLE_TIME));

    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    sync_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sync_msg_size, sync_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sync_msg_size, 0, 1);       /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    };

    last_synced_time = millis();
    
    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;
    
    // if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    // {
    //     uint32_t frame_len;
    
    //     /* Clear good RX frame event in the DW IC status register. */
    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    
    //     /* A frame has been received, read it into the local buffer. */
    //     frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    //     if (frame_len <= sizeof(rx_buffer))
    //     {
    //         dwt_readrxdata(rx_buffer, frame_len, 0);
    //     }
    // } else {
    //     /* Clear RX error/timeout events in the DW IC status register. */
    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    // }
}
