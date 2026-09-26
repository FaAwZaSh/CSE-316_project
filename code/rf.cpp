/*
 * =====================================================================================
 *  PROJECT VANGUARD: Autonomous Sentry Turret - Main Station IFF Challenge Initiator
 * =====================================================================================
 *  MCU:               ATmega32 / ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:         9600 Baud (8-N-1) over USB-to-TTL (PD0/PD1)
 *  Radio:             nRF24L01+ 2.4 GHz Transceiver (SPI + Control Lines)
 * =====================================================================================
 *  ROLE:
 *    Main Turret Station RF Sender. Transmits a rolling cryptographic challenge packet.
 *    Listens for slotted, replay-protected responses from authorized personnel badges.
 *    - Valid Keyed Auth Token received: ACCESS GRANTED -> Friendly Detected
 *    - Timeout (40 ms) / Bad Token:    INTRUDER ALERT -> Hostile Lock / Alarm Trigger
 * =====================================================================================
 * 
 *  ATMEGA32A PIN CONNECTIONS (DIP-40):
 *  -----------------------------------
 *  CE        : Pin 37 (PA3)
 *  CSN       : Pin 38 (PA2)
 *  IRQ       : Pin 36 (PA4)
 *  SS        : Pin 5  (PB4) -> Configured as Output (SPI Master)
 *  MOSI      : Pin 6  (PB5)
 *  MISO      : Pin 7  (PB6)
 *  SCK       : Pin 8  (PB7)
 *  Heartbeat : Pin 1  (PB0)
 *  Alert LED : Pin 33 (PA7)
 * =====================================================================================
 */

#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include "vanguard_rf_protocol.h"

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

#define LED_HEARTBEAT_DDR   DDRB
#define LED_HEARTBEAT_PORT  PORTB
#define LED_HEARTBEAT_BIT   PB0  // Pin 1

#define LED_ALERT_DDR       DDRA
#define LED_ALERT_PORT      PORTA
#define LED_ALERT_BIT       PA7  // Pin 33

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

// ---------- State & Statistics Tracking ----------
static uint16_t global_seq = 0;
static uint32_t total_challenges = 0;
static uint32_t authorized_count = 0;
static uint32_t intruder_count = 0;

// ---------- SPI Low-Level Primitives ----------
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

// ---------- Hardware GPIO & SPI Initialization ----------
void init_hardware() {
  // 1. Control Pins
  CSN_PORT |= (1 << CSN_BIT);
  CSN_DDR  |= (1 << CSN_BIT);

  CE_PORT &= ~(1 << CE_BIT);
  CE_DDR  |= (1 << CE_BIT);

  // IRQ pin as input with pull-up
  IRQ_DDR  &= ~(1 << IRQ_BIT);
  IRQ_PORT |= (1 << IRQ_BIT);

  // Status LEDs
  LED_HEARTBEAT_DDR  |= (1 << LED_HEARTBEAT_BIT);
  LED_HEARTBEAT_PORT &= ~(1 << LED_HEARTBEAT_BIT);

  LED_ALERT_DDR  |= (1 << LED_ALERT_BIT);
  LED_ALERT_PORT &= ~(1 << LED_ALERT_BIT);

  csn_high();
  ce_low();

  // 2. SPI Bus Pins
  SPI_PORT |= (1 << SS_BIT) | (1 << MISO_BIT);
  SPI_DDR  |= (1 << MOSI_BIT) | (1 << SCK_BIT) | (1 << SS_BIT);
  SPI_DDR  &= ~(1 << MISO_BIT);

  // 3. Configure Hardware SPI Master Mode @ F_CPU / 4 (4 MHz @ 16 MHz F_CPU)
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR &= ~(1 << SPI2X);

  // Clear SPI status
  uint8_t dummy = SPSR;
  dummy = SPDR;
  (void)dummy;
}

// ---------- Radio Configuration (VANGUARD Station) ----------
bool init_nrf24_station() {
  ce_low();
  _delay_ms(100);

  nrf_write_reg(NRF_RF_CH, VANGUARD_RF_CHANNEL);         // Channel 76 (2.476 GHz)
  nrf_write_reg(NRF_RF_SETUP, VANGUARD_RF_SETUP_VAL);     // 250 kbps, 0 dBm (Max Power), High LNA
  nrf_write_reg(NRF_SETUP_AW, 0x03);                     // 5-byte address width

  nrf_write_buf(NRF_TX_ADDR, VANGUARD_RF_ADDR, 5);       // "VANG1"
  nrf_write_buf(NRF_RX_ADDR_P0, VANGUARD_RF_ADDR, 5);

  // Enable Auto-ACK & 5 Retries (Forum Proven Approach)
  nrf_write_reg(NRF_EN_AA, 0x01);                        // Auto-ACK enabled on Pipe 0
  nrf_write_reg(NRF_SETUP_RETR, 0x55);                   // 1500 us delay, 5 hardware retries
  nrf_write_reg(NRF_EN_RXADDR, 0x01);                    // Pipe 0 enabled
  nrf_write_reg(NRF_RX_PW_P0, 32);                       // 32-byte fixed payload width

  nrf_flush_tx();
  nrf_flush_rx();
  nrf_clear_interrupts();

  // Power Up in Standby-I mode (PWR_UP=1, PRIM_RX=0, 2-byte CRC)
  nrf_write_reg(NRF_CONFIG, 0x0E);
  _delay_ms(5);

  uint8_t cfg = nrf_read_reg(NRF_CONFIG);
  uint8_t ch  = nrf_read_reg(NRF_RF_CH);
  uint8_t setup_val = nrf_read_reg(NRF_RF_SETUP);

  return (cfg == 0x0E && ch == VANGUARD_RF_CHANNEL && setup_val == VANGUARD_RF_SETUP_VAL);
}

// ---------- Switch Radio to RX Mode ----------
void set_station_rx_mode() {
  ce_low();
  nrf_flush_rx();
  nrf_clear_interrupts();
  nrf_write_reg(NRF_CONFIG, 0x0F); // PRIM_RX=1, PWR_UP=1
  ce_high();
  _delay_us(130); // RX settling time
}

// ---------- Switch Radio to TX Standby Mode ----------
void set_station_tx_mode() {
  ce_low();
  nrf_write_reg(NRF_CONFIG, 0x0E); // PRIM_RX=0, PWR_UP=1
  _delay_us(130);
}

// ---------- Transmit a 32-Byte Packet ----------
// Keeps CE HIGH until transmission completes (prevents clone chip abort)
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

  // Wait for TX_DS or MAX_RT
  uint16_t timeout = 3000;
  bool success = false;
  while (timeout > 0) {
    uint8_t st = nrf_read_reg(NRF_STATUS);
    if (st & (1 << 5)) { // TX_DS: ACK received or packet sent!
      success = true;
      break;
    }
    if (st & (1 << 4)) { // MAX_RT: Max retries exceeded
      break;
    }
    _delay_us(5);
    timeout--;
  }

  ce_low();
  nrf_clear_interrupts();
  return success;
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

// ---------- Read 32-Byte Packet from RX FIFO ----------
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

// =====================================================================================
//  Execute Full IFF Challenge-Response Handshake
// =====================================================================================
void execute_iff_challenge() {
  total_challenges++;
  global_seq++;

  // Toggle Heartbeat LED
  LED_HEARTBEAT_PORT ^= (1 << LED_HEARTBEAT_BIT);

  // 1. Generate Rolling Nonce (Cryptographic Replay Protection)
  uint32_t nonce = ((uint32_t)micros() << 16) | (uint16_t)rand();
  if (nonce == 0) nonce = 0xA5A55A5A;

  // 2. Prepare IFF Challenge Packet (Station -> Badge)
  IFFChallengePacket challenge;
  memset(&challenge, 0, sizeof(challenge));
  challenge.packet_type    = PACKET_TYPE_CHALLENGE;
  challenge.station_id     = 0x01; // Sentry Turret Main Station
  challenge.session_seq    = global_seq;
  challenge.nonce          = nonce;
  challenge.station_uptime = millis();
  strncpy(challenge.magic, "VANGUARD", 8);

  Serial.print(F("\r\n[IFF CHECK] >>> Transmitting Challenge #"));
  Serial.print(global_seq);
  Serial.print(F(" (Nonce: 0x"));
  Serial.print(nonce, HEX);
  Serial.print(F(")... "));

  // 3. Send Challenge Packet over 2.4 GHz RF (250 kbps, 0 dBm)
  set_station_tx_mode();
  bool tx_ok = send_packet_raw(&challenge);

  if (tx_ok) {
    Serial.println(F("[RF TX OK / ACK Received]"));
  } else {
    Serial.println(F("[RF TX Sent - Waiting for Slotted Reply]"));
  }

  // 4. Immediately flip to RX mode & listen for slotted badge responses (<= 40 ms)
  set_station_rx_mode();
  uint32_t window_start_ms = millis();
  bool badge_authenticated = false;
  IFFResponsePacket response;
  memset(&response, 0, sizeof(response));

  while ((millis() - window_start_ms) <= 45) {
    if (has_incoming_packet()) {
      read_packet_raw(&response);

      // Validate packet type, rolling sequence number, and echo nonce
      if (response.packet_type == PACKET_TYPE_RESPONSE &&
          response.session_seq == global_seq &&
          response.nonce_echo  == nonce) {

        // Validate Keyed Cryptographic Signature: Token == Nonce ^ SECRET_KEY
        uint32_t expected_auth = nonce ^ VANGUARD_SECRET_KEY;
        if (response.auth_token == expected_auth) {
          badge_authenticated = true;
          break;
        }
      }
    }
  }

  uint32_t elapsed_ms = millis() - window_start_ms;
  set_station_tx_mode(); // Return to Standby mode

  // 5. Evaluate Result & Update Telemetry
  if (badge_authenticated) {
    authorized_count++;
    LED_ALERT_PORT &= ~(1 << LED_ALERT_BIT); // Alert LED OFF

    char safe_callsign[9] = {0};
    memcpy(safe_callsign, response.badge_callsign, 8);
    safe_callsign[8] = '\0';

    Serial.print(F("  >>> [ACCESS GRANTED] Friendly Badge #"));
    Serial.print(response.badge_id);
    Serial.print(F(" ('"));
    Serial.print(safe_callsign);
    Serial.print(F("') Verified in "));
    Serial.print(elapsed_ms);
    Serial.println(F(" ms!"));
    Serial.println(F("  >>> Status: FRIENDLY IDENTIFIED - Weapons Hold / Passive Tracking"));
  } else {
    intruder_count++;
    LED_ALERT_PORT |= (1 << LED_ALERT_BIT); // Alert LED ON

    Serial.print(F("  >>> [INTRUDER ALERT] No Valid Badge Responded in 45 ms Window! (Elapsed: "));
    Serial.print(elapsed_ms);
    Serial.println(F(" ms)"));
    Serial.println(F("  >>> Status: UNIDENTIFIED TARGET - Hostile Lock / Defensive Engagement"));
  }

  Serial.print(F("  [IFF Telemetry] Challenges: "));
  Serial.print(total_challenges);
  Serial.print(F(" | Authorized: "));
  Serial.print(authorized_count);
  Serial.print(F(" | Breaches/Intruders: "));
  Serial.println(intruder_count);
}

// =====================================================================================
//  Arduino Standard Setup & Loop
// =====================================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 2000);

  Serial.println(F("\r\n========================================================"));
  Serial.println(F("   VANGUARD Autonomous Sentry - IFF Turret Station Node "));
  Serial.println(F("========================================================"));
  Serial.println(F("Protocol: VANGUARD Replay-Protected Challenge-Response"));
  Serial.println(F("DataRate: 250 kbps | Power: 0 dBm (MAX) | Retries: 5x"));
  Serial.println(F("Channel : 76 (2.476 GHz) | Timeout: 45 ms"));
  Serial.println(F("MCU     : ATmega32A DIP-40 @ 16.0 MHz"));
  Serial.println(F("--------------------------------------------------------"));

  init_hardware();

  Serial.print(F("Initializing nRF24L01+ Station Transceiver... "));
  if (init_nrf24_station()) {
    Serial.println(F("[ONLINE / READY]"));
  } else {
    Serial.println(F("[HARDWARE ERROR - Check SPI & 3.3V Wiring]"));
  }
  Serial.println(F("--------------------------------------------------------\r\n"));
}

void loop() {
  execute_iff_challenge();
  _delay_ms(1000);
}
