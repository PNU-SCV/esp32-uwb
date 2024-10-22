#include <Arduino.h>
#include "dw3000.h"
#include "SPI.h"

extern SPISettings _fastSPI;

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define TX_ANT_DLY 16372
#define RX_ANT_DLY 16372
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_RX_TO_RESP_TX_DLY_UUS 640 

#define FRAME_CYCLE_TIME 500
#define TIME_SLOT_LENGTH 100
#define TIME_SLOT_COUNT 1

#define ANCHOR_CHAR 'A'

static dwt_config_t config = {
    5,                
    DWT_PLEN_128,     
    DWT_PAC8,         
    9,                
    9,                
    1,                
    DWT_BR_6M8,       
    DWT_PHRMODE_STD,  
    DWT_PHRRATE_STD,  
    (129 + 8 - 8),    
    DWT_STS_MODE_OFF, 
    DWT_STS_LEN_64,   
    DWT_PDOA_M0       
};

static uint8_t rx_time_sync_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'S', 'Y', 'N', 0xE0, 0, 0};

                  
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', ANCHOR_CHAR, '*', '*', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, '*', '*', 'R', ANCHOR_CHAR, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern dwt_txconfig_t txconfig_options;

void setup()
{
  UART_init();

  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); 

  while (!dwt_checkidlerc()) 
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

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if (dwt_configure(&config)) 
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  dwt_configuretxrf(&txconfig_options);

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range TX");
  Serial.println("Setup over........");
}

void loop()
{
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)));

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_poll_msg, 7) == 0 && rx_buffer[9] == 0xE0) 
      {
        uint32_t resp_tx_time;
        int ret;

        tx_resp_msg[5] = rx_buffer[7];
        tx_resp_msg[6] = rx_buffer[8];
        
        poll_rx_ts = get_rx_timestamp_u64();

        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); 
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);          
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        if (ret == DWT_SUCCESS)
        {
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
          {
          };

          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

          frame_seq_nb = (frame_seq_nb + 1) % 256;
        }
      }
    }
  }
  else
  {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}
