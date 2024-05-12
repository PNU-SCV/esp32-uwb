#include <Arduino.h>
#include "dw3000.h"

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


/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                  /* Channel number. */
    DWT_PLEN_128,       /* Preamble length. Used in TX only. */
    DWT_PAC8,           /* Preamble acquisition chunk size. Used in RX only. */
    9,                  /* TX preamble code. Used in TX only. */
    9,                  /* RX preamble code. Used in RX only. */
    3,                  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,         /* Data rate. */
    DWT_PHRMODE_STD,    /* PHY header mode. */
    DWT_PHRRATE_STD,    /* PHY header rate. */
    (128 + 1 + 8 - 8),  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,   /* STS Off */
    DWT_STS_LEN_128,    /* Ignore value when STS is disabled */
    DWT_PDOA_M0         /* PDOA mode off */
};

static uint8_t tx_poll_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', 'A', 0xE0, 0, 0};
static uint8_t tx_poll_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', 'B', 0xE0, 0, 0};
static uint8_t rx_resp_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_resp_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance_A, distance_B;
extern dwt_txconfig_t txconfig_options;

void poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance);

void calculate_Distance(uint8_t* buffer, double *distance);

void RTLS_Task(void *parameter)
{
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

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    Serial.println("Range RX");
    Serial.println("Setup over........");

    while (1)
    {
        /* Execute a ranging exchange.
            * The device with the tag address will send a poll message to the device with the anchor address.
            * The anchor will receive the poll message and send a response message to the tag.
            * The tag will receive the response message and calculate the distance between the two devices. */
        poll_And_Recieve(tx_poll_msg_A, rx_resp_msg_A, POLL_MSG_SIZE, RESP_MSG_SIZE, &distance_A);
        Serial.print("Distance A: ");
        Serial.println(distance_A);

        poll_And_Recieve(tx_poll_msg_B, rx_resp_msg_B, POLL_MSG_SIZE, RESP_MSG_SIZE, &distance_B);
        Serial.print("Distance B: ");
        Serial.println(distance_B);
    }
}


void poll_And_Recieve(uint8_t *poll_msg, uint8_t *resp_msg, uint8_t poll_msg_size, uint8_t resp_msg_size, double *distance)
{
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