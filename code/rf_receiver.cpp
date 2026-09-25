/*
 * =====================================================================================
 *  PROJECT VANGUARD: Wearable IFF Security Badge - Responder Node
 * =====================================================================================
 *  MCU:               ATmega32 / ATmega32A (8.0 MHz Internal RC Oscillator)
 *  Baud Rate:         9600 Baud (8-N-1) over USB-to-TTL (PD0/PD1)
 * =====================================================================================
 *  ROLE:
 *    Wearable IFF Badge carried by authorized personnel. Listens continuously
 *    on 2.4 GHz for Station IFF Challenges. Computes cryptographic keyed response
 *    (nonce ^ SECRET_KEY) and transmits slotted reply (Badge ID * 4 ms backoff)
 *    within the Station's 40 ms window to authenticate and prevent warning alarms.
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include "vanguard_rf_protocol.h"

// ---------- Badge Configuration ----------
#define BADGE_ID            1
#define BADGE_CALLSIGN      "VIP_01  "
#define SLOTTED_DELAY_MS    (BADGE_ID * 4) // 4 ms for Badge 1 (per proposal)

// ---------- Pin Definitions (ATmega32 DIP-40) ----------
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
#define SS_BIT    PB4  // Pin 5  (Hardware SS - Must be output!)
#define MOSI_BIT  PB5  // Pin 6  (Hardware MOSI)
#define MISO_BIT  PB6  // Pin 7  (Hardware MISO)
#define SCK_BIT   PB7  // Pin 8  (Hardware SCK)

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

// ---------- Statistics ----------
static uint32_t responses_sent = 0;
static uint32_t last_heartbeat_ms = 0;

// ---------- SPI Low-Level Functions ----------
static inline uint8_t spi_transfer(uint8_t data) {
  SPDR = data;
  uint16_t to = 10000;
  while (!(SPSR & (1 << SPIF)) && --to);
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

// ---------- nRF24L01 Register Helpers ----------
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

// ---------- Hardware Init ----------
void init_hardware() {
  CSN_PORT |= (1 << CSN_BIT);
  CSN_DDR  |= (1 << CSN_BIT);

  CE_PORT &= ~(1 << CE_BIT);
  CE_DDR  |= (1 << CE_BIT);

  IRQ_DDR &= ~(1 << IRQ_BIT);
  IRQ_PORT |= (1 << IRQ_BIT);

  csn_high();
  ce_low();

  SPI_PORT |= (1 << SS_BIT) | (1 << MISO_BIT);
  SPI_DDR  |= (1 << MOSI_BIT) | (1 << SCK_BIT) | (1 << SS_BIT);
  SPI_DDR  &= ~(1 << MISO_BIT);

  // SPI Master Mode @ F_CPU / 4 (2 MHz @ 8 MHz F_CPU)
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR &= ~(1 << SPI2X);

  uint8_t dummy = SPSR;
  dummy = SPDR;
  (void)dummy;
}

// ---------- Switch Radio to Constant RX Mode ----------
void set_badge_rx_mode() {
  ce_low();
  nrf_flush_rx();
  nrf_clear_interrupts();
  nrf_write_reg(NRF_CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1
  ce_high();
  _delay_us(130);
}

// ---------- Switch Radio to TX Mode ----------
void set_badge_tx_mode() {
  ce_low();
  nrf_write_reg(NRF_CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0
  _delay_us(130);
}

// ---------- Radio Configuration ----------
bool init_nrf24_badge() {
  ce_low();
  _delay_ms(100);

  nrf_write_reg(NRF_RF_CH, VANGUARD_RF_CHANNEL);
  nrf_write_reg(NRF_RF_SETUP, VANGUARD_RF_SETUP_VAL); // 1 Mbps, -12 dBm
  nrf_write_reg(NRF_SETUP_AW, 0x03);                 // 5-byte address

  nrf_write_buf(NRF_TX_ADDR, VANGUARD_RF_ADDR, 5);
  nrf_write_buf(NRF_RX_ADDR_P0, VANGUARD_RF_ADDR, 5);

  nrf_write_reg(NRF_EN_AA, 0x00);      // Application-layer protocol
  nrf_write_reg(NRF_EN_RXADDR, 0x01);  // Pipe 0 enabled
  nrf_write_reg(NRF_RX_PW_P0, 32);     // 32-byte fixed payload

  nrf_flush_tx();
  nrf_flush_rx();
  nrf_clear_interrupts();

  set_badge_rx_mode();

  return (nrf_read_reg(NRF_CONFIG) == 0x0F && nrf_read_reg(NRF_RF_CH) == VANGUARD_RF_CHANNEL);
}

// ---------- Check if Packet is in RX FIFO ----------
bool has_incoming_packet() {
  uint8_t status = nrf_read_reg(NRF_STATUS);
  if (status == 0x00 || status == 0xFF) {
    return false; // SPI bus unpowered, floating, or disconnected
  }
  if (status & (1 << 6)) { // RX_DR asserted
    return true;
  }
  uint8_t fifo = nrf_read_reg(NRF_FIFO_STATUS);
  if ((fifo != 0xFF) && !(fifo & 0x01)) { // RX FIFO not empty
    return true;
  }
  return false;
}

// ---------- Read 32-byte Packet from RX FIFO ----------
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

// ---------- Transmit a 32-byte Packet ----------
void send_packet_raw(const void *packet_ptr) {
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

  // Pulse CE >= 10us
  ce_high();
  _delay_us(15);
  ce_low();

  // Wait for TX complete
  uint16_t timeout = 2000;
  while (timeout > 0) {
    uint8_t st = nrf_read_reg(NRF_STATUS);
    if (st & (1 << 5)) { // TX_DS
      break;
    }
    _delay_us(5);
    timeout--;
  }
  nrf_clear_interrupts();
}

void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  Serial.println(F("\r\n=================================================="));
  Serial.println(F("   VANGUARD Wearable IFF Security Badge Node      "));
  Serial.println(F("=================================================="));
  Serial.print(F("Badge ID: "));
  Serial.print(BADGE_ID);
  Serial.print(F(" | Callsign: "));
  Serial.println(F(BADGE_CALLSIGN));
  Serial.print(F("Clock   : 8.0 MHz Internal RC | Baud: 9600 (0.2% err)\r\n"));
  Serial.println(F("Mode    : Slotted Response (Badge ID x 4 ms backoff)"));
  Serial.println(F("Channel : 76 (2.476 GHz) | Rate: 1 Mbps"));
  Serial.println(F("--------------------------------------------------"));

  init_hardware();

  Serial.print(F("Initializing nRF24L01 Badge Node... "));
  bool ok = init_nrf24_badge();
  uint8_t cfg = nrf_read_reg(NRF_CONFIG);
  uint8_t ch = nrf_read_reg(NRF_RF_CH);
  uint8_t st = nrf_read_reg(NRF_STATUS);

  if (ok) {
    Serial.println(F("[ONLINE / LISTENING]"));
  } else {
    Serial.println(F("[HARDWARE ERROR - Check SPI Wiring]"));
  }
  Serial.print(F("SPI Diagnostic -> CONFIG: 0x"));
  Serial.print(cfg, HEX);
  Serial.print(F(" | RF_CH: "));
  Serial.print(ch);
  Serial.print(F(" | STATUS: 0x"));
  Serial.println(st, HEX);

  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("Standing by for Turret IFF Challenges...\r\n"));
}

void loop() {
  if (has_incoming_packet()) {
    IFFChallengePacket challenge;
    read_packet_raw(&challenge);

    // Verify magic header and packet type
    if (challenge.packet_type == PACKET_TYPE_CHALLENGE &&
        strncmp(challenge.magic, "VANGUARD", 8) == 0) {

      // 1. Calculate slotted multi-badge response backoff delay (4 ms)
      _delay_ms(SLOTTED_DELAY_MS);

      // 2. Compute Replay-Protected Keyed Auth Token
      uint32_t auth_token = challenge.nonce ^ VANGUARD_SECRET_KEY;

      // 3. Populate Response Packet
      IFFResponsePacket response;
      memset(&response, 0, sizeof(response));
      response.packet_type    = PACKET_TYPE_RESPONSE;
      response.badge_id       = BADGE_ID;
      response.session_seq    = challenge.session_seq;
      response.nonce_echo     = challenge.nonce;
      response.auth_token     = auth_token;
      response.badge_uptime   = millis();
      strncpy(response.badge_callsign, BADGE_CALLSIGN, 8);
      response.status_flags   = 0x00; // OK / Active

      // 4. Send Response
      set_badge_tx_mode();
      send_packet_raw(&response);

      // 5. Instantly return to listening mode
      set_badge_rx_mode();

      responses_sent++;
      Serial.print(F("[IFF RESPONDED #"));
      Serial.print(responses_sent);
      Serial.print(F("] Seq: "));
      Serial.print(challenge.session_seq);
      Serial.print(F(" | Nonce: 0x"));
      Serial.print(challenge.nonce, HEX);
      Serial.print(F(" | Token: 0x"));
      Serial.print(auth_token, HEX);
      Serial.println(F(" -> Response Transmitted (Access Granted Expected)"));
    }
  }

  // Heartbeat every 3 seconds if idle
  if (millis() - last_heartbeat_ms > 3000) {
    last_heartbeat_ms = millis();
    Serial.print(F("[Badge Heartbeat] Standing by on Ch76 | Responses Sent: "));
    Serial.println(responses_sent);
  }
}
