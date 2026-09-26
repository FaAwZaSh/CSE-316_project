/*
 * =====================================================================================
 *  PROJECT VANGUARD: nRF24L01+ 2.4 GHz Receiver & IFF Responder Node
 * =====================================================================================
 *  MCU:               ATmega32 / ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:         9600 Baud (8-N-1) over USB-to-TTL (PD0/PD1)
 *  Radio:             nRF24L01+ Transceiver (SPI + Control Lines)
 * =====================================================================================
 *
 *  PIN DEFINITIONS (ATmega32 DIP-40):
 *  ----------------------------------
 *  CE        : Pin 37 (PA3)
 *  CSN       : Pin 38 (PA2)
 *  IRQ       : Pin 36 (PA4)
 *  SS        : Pin 5  (PB4) -> Configured as Output (SPI Master)
 *  MOSI      : Pin 6  (PB5)
 *  MISO      : Pin 7  (PB6)
 *  SCK       : Pin 8  (PB7)
 *  Status LED: Pin 1  (PB0)
 *  USART RXD : Pin 14 (PD0)
 *  USART TXD : Pin 15 (PD1)
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "vanguard_rf_protocol.h"

// ---------- Pin Definitions ----------
#define CE_PORT   PORTA
#define CE_DDR    DDRA
#define CE_BIT    PA3  // Pin 37

#define CSN_PORT  PORTA
#define CSN_DDR   DDRA
#define CSN_BIT   PA2  // Pin 38

#define IRQ_PIN   PINA
#define IRQ_DDR   DDRA
#define IRQ_PORT  PORTA
#define IRQ_BIT   PA4  // Pin 36

#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SS_BIT    PB4  // Pin 5  (Hardware SS - Must be output)
#define MOSI_BIT  PB5  // Pin 6  (Hardware MOSI)
#define MISO_BIT  PB6  // Pin 7  (Hardware MISO)
#define SCK_BIT   PB7  // Pin 8  (Hardware SCK)

#define LED_PORT  PORTB
#define LED_DDR   DDRB
#define LED_BIT   PB0  // Pin 1 (Status / Heartbeat LED)

// ---------- nRF24L01 Commands & Registers ----------
#define NRF_R_REGISTER      0x00
#define NRF_W_REGISTER      0x20
#define NRF_R_RX_PAYLOAD    0x61
#define NRF_W_TX_PAYLOAD    0xA0
#define NRF_FLUSH_TX        0xE1
#define NRF_FLUSH_RX        0xE2
#define NRF_NOP             0xFF

#define NRF_CONFIG          0x00
#define NRF_EN_AA           0x01
#define NRF_EN_RXADDR       0x02
#define NRF_SETUP_AW        0x03
#define NRF_SETUP_RETR      0x04
#define NRF_RF_CH           0x05
#define NRF_RF_SETUP        0x06
#define NRF_STATUS          0x07
#define NRF_RX_ADDR_P0      0x0A
#define NRF_TX_ADDR         0x10
#define NRF_RX_PW_P0        0x11
#define NRF_FIFO_STATUS     0x17

// ---------- Badge Parameters ----------
#define BADGE_ID            1
#define BADGE_CALLSIGN      "VIP_01  "
#define SLOTTED_DELAY_MS    (BADGE_ID * 4) // 4 ms slotted backoff for Badge 1

// ---------- State Tracking ----------
static uint32_t packets_received = 0;
static uint32_t responses_sent = 0;
static uint32_t last_heartbeat_ms = 0;

// ---------- SPI Primitives ----------
static inline uint8_t spi_transfer(uint8_t data) {
  SPDR = data;
  uint16_t timeout = 10000;
  while (!(SPSR & (1 << SPIF)) && --timeout);
  return SPDR;
}

static inline void csn_low() {
  CSN_PORT &= ~(1 << CSN_BIT);
  _delay_us(5);
}

static inline void csn_high() {
  _delay_us(5);
  CSN_PORT |= (1 << CSN_BIT);
  _delay_us(5);
}

static inline void ce_low()  { CE_PORT &= ~(1 << CE_BIT); }
static inline void ce_high() { CE_PORT |= (1 << CE_BIT); }

// ---------- nRF24 Register Access ----------
uint8_t nrf_read_reg(uint8_t reg) {
  csn_low();
  spi_transfer(NRF_R_REGISTER | (reg & 0x1F));
  uint8_t val = spi_transfer(NRF_NOP);
  csn_high();
  return val;
}

uint8_t nrf_write_reg(uint8_t reg, uint8_t val) {
  csn_low();
  uint8_t status = spi_transfer(NRF_W_REGISTER | (reg & 0x1F));
  spi_transfer(val);
  csn_high();
  return status;
}

void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len) {
  csn_low();
  spi_transfer(NRF_W_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer(buf[i]);
  }
  csn_high();
}

void nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len) {
  csn_low();
  spi_transfer(NRF_R_REGISTER | (reg & 0x1F));
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = spi_transfer(NRF_NOP);
  }
  csn_high();
}

void nrf_flush_tx() {
  csn_low();
  spi_transfer(NRF_FLUSH_TX);
  csn_high();
}

void nrf_flush_rx() {
  csn_low();
  spi_transfer(NRF_FLUSH_RX);
  csn_high();
}

void nrf_clear_interrupts() {
  nrf_write_reg(NRF_STATUS, (1 << 6) | (1 << 5) | (1 << 4));
}

// ---------- Radio Modes ----------
void set_rx_mode() {
  ce_low();
  nrf_flush_rx();
  nrf_clear_interrupts();
  nrf_write_reg(NRF_CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1, 2-byte CRC
  ce_high();
  _delay_us(130);
}

void set_tx_mode() {
  ce_low();
  nrf_write_reg(NRF_CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0, 2-byte CRC
  _delay_us(130);
}

// ---------- Hardware Init ----------
void init_hardware() {
  // 1. Control Pins
  CSN_PORT |= (1 << CSN_BIT);
  CSN_DDR  |= (1 << CSN_BIT);

  CE_PORT &= ~(1 << CE_BIT);
  CE_DDR  |= (1 << CE_BIT);

  // IRQ pin with internal pull-up
  IRQ_DDR  &= ~(1 << IRQ_BIT);
  IRQ_PORT |= (1 << IRQ_BIT);

  // Status LED on PB0
  LED_DDR  |= (1 << LED_BIT);
  LED_PORT &= ~(1 << LED_BIT);

  csn_high();
  ce_low();

  // 2. SPI Bus Pins (Master)
  SPI_PORT |= (1 << SS_BIT) | (1 << MISO_BIT);
  SPI_DDR  |= (1 << MOSI_BIT) | (1 << SCK_BIT) | (1 << SS_BIT);
  SPI_DDR  &= ~(1 << MISO_BIT);

  // Hardware SPI Master @ F_CPU / 4 (2 MHz @ 8 MHz F_CPU)
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR &= ~(1 << SPI2X);

  uint8_t dummy = SPSR;
  dummy = SPDR;
  (void)dummy;
}

// ---------- Radio Init ----------
bool init_nrf24_receiver() {
  ce_low();
  _delay_ms(100);

  nrf_write_reg(NRF_RF_CH, VANGUARD_RF_CHANNEL);       // 76 (2.476 GHz)
  nrf_write_reg(NRF_RF_SETUP, VANGUARD_RF_SETUP_VAL);   // 250 kbps, 0 dBm (Max Power), High LNA
  nrf_write_reg(NRF_SETUP_AW, 0x03);                   // 5-byte address width

  nrf_write_buf(NRF_TX_ADDR, VANGUARD_RF_ADDR, 5);     // "VANG1"
  nrf_write_buf(NRF_RX_ADDR_P0, VANGUARD_RF_ADDR, 5);

  // Disable Auto-ACK (clone nRF24 modules have broken Enhanced ShockBurst)
  nrf_write_reg(NRF_EN_AA, 0x00);        // Auto-ACK DISABLED
  nrf_write_reg(NRF_SETUP_RETR, 0x00);   // No retries
  nrf_write_reg(NRF_EN_RXADDR, 0x01);    // Enable Pipe 0
  nrf_write_reg(NRF_RX_PW_P0, 32);       // 32-byte payload

  nrf_flush_tx();
  nrf_flush_rx();
  nrf_clear_interrupts();

  set_rx_mode();

  uint8_t cfg = nrf_read_reg(NRF_CONFIG);
  uint8_t ch  = nrf_read_reg(NRF_RF_CH);
  return (cfg == 0x0F && ch == VANGUARD_RF_CHANNEL);
}

// ---------- Packet Check & Receive ----------
bool has_incoming_packet() {
  uint8_t status = nrf_read_reg(NRF_STATUS);
  if (status == 0x00 || status == 0xFF) return false;
  if (status & (1 << 6)) return true; // RX_DR
  uint8_t fifo = nrf_read_reg(NRF_FIFO_STATUS);
  if ((fifo != 0xFF) && !(fifo & 0x01)) return true; // RX FIFO not empty
  return false;
}

void read_packet_raw(void *buf) {
  csn_low();
  spi_transfer(NRF_R_RX_PAYLOAD);
  uint8_t *p = (uint8_t *)buf;
  for (uint8_t i = 0; i < 32; i++) {
    p[i] = spi_transfer(NRF_NOP);
  }
  csn_high();
  nrf_clear_interrupts();
  nrf_flush_rx();
}

bool send_packet_raw(const void *packet_ptr) {
  ce_low();
  nrf_flush_tx();
  nrf_clear_interrupts();

  csn_low();
  spi_transfer(NRF_W_TX_PAYLOAD);
  const uint8_t *p = (const uint8_t *)packet_ptr;
  for (uint8_t i = 0; i < 32; i++) {
    spi_transfer(p[i]);
  }
  csn_high();

  // Hold CE high while transmitting
  ce_high();

  uint16_t timeout = 3000;
  bool success = false;
  while (timeout > 0) {
    uint8_t st = nrf_read_reg(NRF_STATUS);
    if (st & (1 << 5)) { // TX_DS
      success = true;
      break;
    }
    if (st & (1 << 4)) { // MAX_RT
      break;
    }
    _delay_us(5);
    timeout--;
  }

  ce_low();
  nrf_clear_interrupts();
  return success;
}

// ---------- Arduino Setup & Loop ----------
void setup() {
  // Calibrate 8 MHz Internal RC Oscillator for exact 9600 Baud timing
  OSCCAL = 0xA2;

  Serial.begin(9600);
  while (!Serial && millis() < 1500);

  Serial.println(F("\r\n========================================================"));
  Serial.println(F("   PROJECT VANGUARD: nRF24L01+ Receiver / Badge Node    "));
  Serial.println(F("========================================================"));
  Serial.println(F("MCU     : ATmega32 / ATmega32A @ 8.0 MHz (Internal RC)"));
  Serial.println(F("Baud    : 9600 (8-N-1) | Calibrated OSCCAL = 0xA2"));
  Serial.println(F("Channel : 76 (2.476 GHz) | Rate: 1 Mbps | Power: -12 dBm"));
  Serial.println(F("Address : 'VANG1' | Fixed Payload: 32 Bytes"));
  Serial.println(F("Role    : Slotted IFF Responder (Badge ID 1, 4 ms backoff)"));
  Serial.println(F("--------------------------------------------------------"));

  init_hardware();

  Serial.print(F("Initializing nRF24L01+ Receiver... "));
  bool ok = init_nrf24_receiver();

  uint8_t cfg = nrf_read_reg(NRF_CONFIG);
  uint8_t ch  = nrf_read_reg(NRF_RF_CH);
  uint8_t st  = nrf_read_reg(NRF_STATUS);

  if (ok) {
    Serial.println(F("[ONLINE / READY]"));
    Serial.println(F("[+] Radio configured and actively listening on Channel 76."));
  } else {
    Serial.println(F("[HARDWARE ERROR - Check Wiring & 3.3V Power]"));
  }

  Serial.print(F("SPI Registers -> CONFIG: 0x"));
  Serial.print(cfg, HEX);
  Serial.print(F(" | CH: "));
  Serial.print(ch);
  Serial.print(F(" | STATUS: 0x"));
  Serial.println(st, HEX);
  Serial.println(F("--------------------------------------------------------\r\n"));
  Serial.println(F("Listening for incoming Turret IFF Challenges or RF packets..."));
}

void loop() {
  // 1. Check for incoming RF packet
  if (has_incoming_packet()) {
    uint8_t rx_buf[32];
    read_packet_raw(rx_buf);
    packets_received++;

    // Pulse LED
    LED_PORT |= (1 << LED_BIT);

    Serial.println(F("\r\n--------------------------------------------------------"));
    Serial.print(F(">>> [PACKET #"));
    Serial.print(packets_received);
    Serial.print(F(" RECEIVED] Time: "));
    Serial.print(millis());
    Serial.println(F(" ms"));

    // Check if it is a Vanguard IFF Challenge Packet
    IFFChallengePacket *challenge = (IFFChallengePacket *)rx_buf;
    if (challenge->packet_type == PACKET_TYPE_CHALLENGE &&
        strncmp(challenge->magic, "VANGUARD", 8) == 0) {

      // Cache challenge fields before overwriting rx_buf
      uint8_t  c_station_id   = challenge->station_id;
      uint16_t c_session_seq  = challenge->session_seq;
      uint32_t c_nonce        = challenge->nonce;
      uint32_t c_uptime       = challenge->station_uptime;

      // 1. Slotted backoff delay (4 ms for Badge 1)
      _delay_ms(SLOTTED_DELAY_MS);

      // 2. Compute Keyed Authentication Token: Nonce ^ Secret Key
      uint32_t auth_token = c_nonce ^ VANGUARD_SECRET_KEY;

      // 3. Prepare IFF Response Packet
      IFFResponsePacket response;
      memset(&response, 0, sizeof(response));
      response.packet_type    = PACKET_TYPE_RESPONSE;
      response.badge_id       = BADGE_ID;
      response.session_seq    = c_session_seq;
      response.nonce_echo     = c_nonce;
      response.auth_token     = auth_token;
      response.badge_uptime   = millis();
      strncpy(response.badge_callsign, BADGE_CALLSIGN, 8);
      response.status_flags   = 0x00; // Normal / Active

      // 4. Send Response back to Turret Station FIRST (time-critical!)
      set_tx_mode();
      bool tx_ok = send_packet_raw(&response);
      set_rx_mode(); // Immediately return to listening

      responses_sent++;

      // 5. Debug output AFTER response is sent (no longer blocks the TX path)
      Serial.println(F("    Packet Type : VANGUARD IFF CHALLENGE (Station -> Badge)"));
      Serial.print(F("    Station ID  : ")); Serial.println(c_station_id);
      Serial.print(F("    Session Seq : ")); Serial.println(c_session_seq);
      Serial.print(F("    Challenge Nonce: 0x")); Serial.println(c_nonce, HEX);
      Serial.print(F("    Station Up  : ")); Serial.print(c_uptime); Serial.println(F(" ms"));
      Serial.print(F("    [IFF RESPONDED #"));
      Serial.print(responses_sent);
      Serial.print(F("] Reply Sent! Token: 0x"));
      Serial.print(auth_token, HEX);
      if (tx_ok) {
        Serial.println(F(" [TX OK / ACK Received]"));
      } else {
        Serial.println(F(" [TX Sent - No ACK]"));
      }
    }
    // Check if it is an IFF Response Packet
    else if (rx_buf[0] == PACKET_TYPE_RESPONSE) {
      IFFResponsePacket *resp = (IFFResponsePacket *)rx_buf;
      Serial.println(F("    Packet Type : VANGUARD IFF RESPONSE (Badge -> Station)"));
      Serial.print(F("    Badge ID    : ")); Serial.println(resp->badge_id);
      Serial.print(F("    Callsign    : "));
      for (int i = 0; i < 8; i++) if (resp->badge_callsign[i]) Serial.print(resp->badge_callsign[i]);
      Serial.println();
      Serial.print(F("    Nonce Echo  : 0x")); Serial.println(resp->nonce_echo, HEX);
      Serial.print(F("    Auth Token  : 0x")); Serial.println(resp->auth_token, HEX);
    }
    // Generic or Text Packet
    else {
      Serial.println(F("    Packet Type : Raw Payload / Test Data"));
      Serial.print(F("    Text/ASCII  : \""));
      for (int i = 0; i < 32; i++) {
        char c = (char)rx_buf[i];
        if (c >= 32 && c <= 126) Serial.print(c);
        else if (c == 0) break;
        else Serial.print('.');
      }
      Serial.println(F("\""));
    }

    // Print Raw Hex
    Serial.print(F("    RAW HEX     : "));
    for (int i = 0; i < 32; i++) {
      if (rx_buf[i] < 0x10) Serial.print('0');
      Serial.print(rx_buf[i], HEX);
      Serial.print(' ');
      if (i == 15) Serial.print(F("  "));
    }
    Serial.println();
    Serial.println(F("--------------------------------------------------------\r\n"));

    _delay_ms(20);
    LED_PORT &= ~(1 << LED_BIT);
  }

  // 2. Periodic Heartbeat (every 3 seconds)
  if (millis() - last_heartbeat_ms > 3000) {
    last_heartbeat_ms = millis();

    // Heartbeat blink on PB0
    LED_PORT |= (1 << LED_BIT);
    _delay_ms(20);
    LED_PORT &= ~(1 << LED_BIT);

    Serial.print(F("[Badge Heartbeat] Standing by on Ch 76 | Packets: "));
    Serial.print(packets_received);
    Serial.print(F(" | Responses: "));
    Serial.println(responses_sent);
  }
}
