#include <Arduino.h>
#include "dw3000.h"
#include "rtls.h"

const Point3D anchor_A = {0, 2, 0}, anchor_B = {0, 2, 5};

Point2D tag_position = {0, 0};

uint32_t lastSyncedTime = 0;

static uint8_t tx_time_sync_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'S', 'Y', 'N', 0xE0, 0, 0};

static uint8_t tx_poll_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'A', 'T', 'A', 0xE0, 0, 0};
static uint8_t tx_poll_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'B', 'T', 'A', 0xE0, 0, 0};
static uint8_t rx_resp_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'A', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'A', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance_A, distance_B;

uint32_t getCurrentTime(void)
{
    return millis() - lastSyncedTime;
}

void poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance, uint8_t time_slot_idx);

void calculate_Distance(uint8_t* buffer, double *distance);

void broadcast_Time_Sync_Msg(uint8_t *sync_msg, uint8_t sync_msg_size, uint8_t time_slot_idx);

void calculate_Position(void);

void RTLS_Task(void *parameter)
{
    Serial.println("Range RX");
    Serial.println("Setup over........");

    while (1)
    {
        /* Broadcast time sync message */
        broadcast_Time_Sync_Msg(tx_time_sync_msg, sizeof(tx_time_sync_msg), TIME_SYNC_IDX);

        /* Execute a ranging exchange.
            * The device with the tag address will send a poll message to the device with the anchor address.
            * The anchor will receive the poll message and send a response message to the tag.
            * The tag will receive the response message and calculate the distance between the two devices. */
        poll_And_Recieve(tx_poll_msg_A, rx_resp_msg_A, POLL_MSG_SIZE, RESP_MSG_SIZE, &distance_A, TIME_SLOT_IDX_0);
        Serial.print("Distance A: ");
        Serial.println(distance_A);

        poll_And_Recieve(tx_poll_msg_B, rx_resp_msg_B, POLL_MSG_SIZE, RESP_MSG_SIZE, &distance_B, TIME_SLOT_IDX_1);
        Serial.print("Distance B: ");
        Serial.println(distance_B);

        /* Calulate current position rest of time slots (and, post call end of the function)*/
        calculate_Position();
    }
}


void poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance, uint8_t time_slot_idx)
{
    /* Polling for TDMA TIME SLOT */
    //while(((getCurrentTime() % TIME_SLOT_SEQ_LENTH) / TIME_SLOT_LENGTH) != time_slot_idx);

    /* Delay for TDMA Time Slot */
    vTaskDelay(pdMS_TO_TICKS((FRAME_CYCLE_TIME + time_slot_idx * TIME_SLOT_LENGTH - (getCurrentTime() % FRAME_CYCLE_TIME)) % FRAME_CYCLE_TIME));
    
    /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(poll_msg_size, poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(poll_msg_size, 0, 1);       /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    };
    
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
            }
        }
    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
            
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

    // 거리 계산 (TOF × 빛의 속도)
    *distance = tof * SPEED_OF_LIGHT;
}


void broadcast_Time_Sync_Msg(uint8_t *sync_msg, uint8_t sync_msg_size, uint8_t time_slot_idx)
{
    /* Delay for TDMA time slot */
    vTaskDelay(pdMS_TO_TICKS((FRAME_CYCLE_TIME + time_slot_idx * TIME_SLOT_LENGTH - (getCurrentTime() % FRAME_CYCLE_TIME)) % FRAME_CYCLE_TIME));

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
        }
    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}

void calculate_Position(void) {
    /* 중점 좌표 계산 */
    float xM = (anchor_A.x + anchor_B.y) / 2;
    float yM = (anchor_A.y + anchor_B.y) / 2;
    float zM = (anchor_A.z + anchor_B.z) / 2;

    /* 각 앵커와 중점 사이의 거리 */
    float L = sqrt(pow(anchor_A.x - xM, 2) + pow(anchor_A.y - yM, 2) + pow(anchor_A.z - zM, 2));

    /* dA^2 - dB^2 + L^2 계산 */
    float C = pow(distance_A, 2) - pow(distance_B, 2) + pow(L, 2) * 4;

    /* y-좌표 계산 */
    float y = (C / (2 * (anchor_B.y - anchor_A.y))) - yM;
    float xPart = pow(distance_A, 2) - pow(y + yM - anchor_A.y, 2);
    float zPart = pow(distance_A, 2) - pow(y + yM - anchor_A.y, 2) - pow(xM - anchor_A.x, 2);

    /* x-좌표와 z-좌표 계산 */
    float x = sqrt(xPart) + xM;
    float z = sqrt(zPart) + zM;

    tag_position = {x, z};

    Serial.print("Tag Position: x = ");
    Serial.print(x, 2);
    Serial.print(", y = ");
    Serial.print(y, 2);
    Serial.print(", z = ");
    Serial.println(z, 2);
}