#include "dw3000_device_api.h"
#include "dw3000_config_options.h"
#include "dw3000_port.h"
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

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg_A[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 앵커 A 응답
static uint8_t rx_resp_msg_B[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'B', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 앵커 B 응답
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double distance_A, distance_B;  // 각 앵커와의 거리
extern dwt_txconfig_t txconfig_options;

bool receive_response(uint8_t* expected_msg, double* distance);

void calculate_distance(uint8_t* buffer, double* distance);

void estimate_position(double distance_A, double distance_B);

void setup()
{
  UART_init();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
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
}

void loop()
{
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  // 앵커 A와 B로부터의 응답 수신 및 처리
    if (receive_response(rx_resp_msg_A, &distance_A) && receive_response(rx_resp_msg_B, &distance_B)) {
        Serial.println("Received response from both anchors");
        Serial.println("Distance A: " + String(distance_A) + "m");
        Serial.println("Distance B: " + String(distance_B) + "m");
        estimate_position(distance_A, distance_B);
    }

  /* Execute a delay between ranging exchanges. */
  Sleep(RNG_DELAY_MS);
}

bool receive_response(uint8_t* expected_msg, double* distance) {
    if ((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_RXFCG_BIT_MASK) {
        uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, expected_msg, ALL_MSG_COMMON_LEN) == 0) {
                calculate_distance(rx_buffer, distance);
                return true;
            }
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    }
    return false;
}

void calculate_distance(uint8_t* buffer, double* distance) {
    // 거리 계산 로직
    // 특정 앵커로부터의 거리를 계산하여 distance에 저장
}

void estimate_position(double distance_A, double distance_B) {
    // 삼각 측량을 사용하여 위치 추정
    // 위치 정보 출력 또는 저장
}