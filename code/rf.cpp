/*
 * =====================================================================================
 *  PROJECT VANGUARD: Autonomous Sentry Turret - Main Station IFF Challenge Initiator
 * =====================================================================================
 *  MCU:               ATmega32 / ATmega32A (16.0 MHz External Crystal)
 *  Baud Rate:         9600 Baud (8-N-1) over USB-to-TTL (PD0/PD1)
 *  Radio:             nRF24L01+ 2.4 GHz Transceiver (SPI + Control Lines)
 * =====================================================================================
 *  ROLE:
 *    Main Turret Station RF Sender. When triggered (or during periodic sector watch),
 *    transmits a rolling cryptographic challenge packet (32 bytes fixed).
 *    Immediately opens a strict 40 ms RX window to receive slotted, replay-protected
 *    responses from authorized personnel badges (slotted backoff = Badge ID * 4 ms).
 *    - Valid Keyed Auth Token received: ACCESS GRANTED -> Friendly Detected
 *    - Timeout (40 ms) / Bad Token:    INTRUDER ALERT -> Hostile Lock / Alarm Trigger
 * =====================================================================================
 * 
 *  ATMEGA32A CONNECTION CONFIGURATION (DIP-40 PINOUT):
 *  ---------------------------------------------------
 * 
 *                     +---[ \_/ ]---+
 *     (Heartbeat)PB0 1 |             | 40  PA0
 *                PB1 2 |             | 39  PA1
 *                PB2 3 |             | 38  PA2 (CSN - nRF24L01 Chip Select Not)
 *                PB3 4 |             | 37  PA3 (CE  - nRF24L01 Chip Enable)
 *        (!SS)   PB4 5 |             | 36  PA4 (IRQ - nRF24L01 Interrupt / Pull-up)
 *        (MOSI)  PB5 6 |   ATmega32A | 35  PA5
 *        (MISO)  PB6 7 |    DIP-40   | 34  PA6
 *        (SCK)   PB7 8 |             | 33  PA7 (Alert / Lock Status LED)
 *             !RESET 9 |             | 32  AREF
 *                VCC 10|             | 31  GND
 *                GND 11|             | 30  AVCC
 *              XTAL2 12|             | 29  PC7
 *              XTAL1 13|             | 28  PC6
 *         (RXD)  PD0 14|             | 27  PC5
 *         (TXD)  PD1 15|             | 26  PC4
 *                PD2 16|             | 25  PC3
 *                PD3 17|             | 24  PC2
 *                PD4 18|             | 23  PC1
 *                PD5 19|             | 22  PC0
 *                PD6 20|             | 21  PD7
 *                     +-------------+
 * 
 *  nRF24L01+ 8-PIN MODULE PINOUT (TOP VIEW / PINS POINTING DOWN):
 *  --------------------------------------------------------------
 *       +-------+-------+
 *   GND | (1)   (2) | VCC (+3.3V ONLY!)
 *    CE | (3)   (4) | CSN
 *   SCK | (5)   (6) | MOSI
 *  MISO | (7)   (8) | IRQ
 *       +-------+-------+
 * 
 *  HARDWARE WIRING DETAILS:
 *  ------------------------
 *  1. Power & Clock:
 *     - Pin 10 (VCC)    --> +5V
 *     - Pin 11 (GND)    --> Common GND
 *     - Pin 30 (AVCC)   --> +5V (Mandatory for Port A logic)
 *     - Pin 31 (GND)    --> Common GND
 *     - Pin 9  (!RESET) --> 10k resistor pull-up to +5V
 *     - Pin 12 (XTAL2)  --> 16MHz Crystal + 22pF cap to GND
 *     - Pin 13 (XTAL1)  --> 16MHz Crystal + 22pF cap to GND
 * 
 *  2. USB-to-TTL Serial Bridge (PL2303 / CP2102 / CH340 / FTDI):
 *     - USB-TTL TXD     --> ATmega32 Pin 14 (PD0 / RXD)
 *     - USB-TTL RXD     --> ATmega32 Pin 15 (PD1 / TXD)
 *     - USB-TTL GND     --> Common GND (Mandatory)
 *     - Baud Rate       --> 9600 Baud (8-N-1)
 * 
 *  3. nRF24L01 2.4 GHz Transceiver Connections:
 *     - Pin 1 (GND)     --> Common GND
 *     - Pin 2 (VCC)     --> +3.3V POWER ONLY!
 *                           * CAUTION: Connecting to 5V will permanently burn the chip!
 *                           * Add a 10uF - 100uF electrolytic capacitor directly across
 *                             VCC and GND pins of the nRF24L01 for power rail stability.
 *     - Pin 3 (CE)      --> ATmega32 Pin 37 (PA3)
 *     - Pin 4 (CSN)     --> ATmega32 Pin 38 (PA2)
 *     - Pin 5 (SCK)     --> ATmega32 Pin 8  (PB7 / Hardware SCK)
 *     - Pin 6 (MOSI)    --> ATmega32 Pin 6  (PB5 / Hardware MOSI)
 *     - Pin 7 (MISO)    --> ATmega32 Pin 7  (PB6 / Hardware MISO)
 *     - Pin 8 (IRQ)     --> ATmega32 Pin 36 (PA4 / Input with Internal Pull-up)
 * 
 *  4. Indicators (Optional):
 *     - Pin 1  (PB0)    --> Green Heartbeat LED (+ 330 ohm resistor to GND)
 *     - Pin 33 (PA7)    --> Red Intruder / Lock LED (+ 330 ohm resistor to GND)
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
#define SS_BIT    PB4  // Pin 5  (Hardware SS - Must be configured as output!)
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

  nrf_write_reg(NRF_RF_CH, VANGUARD_RF_CHANNEL);
  nrf_write_reg(NRF_RF_SETUP, VANGUARD_RF_SETUP_VAL); // 1 Mbps, -12 dBm
  nrf_write_reg(NRF_SETUP_AW, 0x03);                 // 5-byte address width

  nrf_write_buf(NRF_TX_ADDR, VANGUARD_RF_ADDR, 5);
  nrf_write_buf(NRF_RX_ADDR_P0, VANGUARD_RF_ADDR, 5);

  nrf_write_reg(NRF_EN_AA, 0x00);      // Custom slotted protocol without auto-ACK collisions
  nrf_write_reg(NRF_EN_RXADDR, 0x01);  // Pipe 0 enabled
  nrf_write_reg(NRF_RX_PW_P0, 32);     // 32-byte fixed payload width

  nrf_flush_tx();
  nrf_flush_rx();
  nrf_clear_interrupts();

  // Power Up in Standby-I mode (PWR_UP=1, PRIM_RX=0, 2-byte CRC)
  nrf_write_reg(NRF_CONFIG, 0x0E);
  _delay_ms(5);

  uint8_t cfg = nrf_read_reg(NRF_CONFIG);
  uint8_t ch  = nrf_read_reg(NRF_RF_CH);

  return (cfg == 0x0E && ch == VANGUARD_RF_CHANNEL);
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

  // Pulse CE for >= 10us to trigger TX
  ce_high();
  _delay_us(15);
  ce_low();

  // Wait for TX FIFO transmission completion
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
  if (nonce == 0) nonce = 0xA5A55A5A; // non-zero fallback

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
  Serial.println(F(")..."));

  // 3. Send Challenge Packet over 2.4 GHz RF
  set_station_tx_mode();
  send_packet_raw(&challenge);

  // 4. Immediately flip to RX mode & listen for slotted badge responses (<= 40 ms)
  set_station_rx_mode();
  uint32_t window_start_ms = millis();
  bool badge_authenticated = false;
  bool retried = false;
  IFFResponsePacket response;
  memset(&response, 0, sizeof(response));

  while ((millis() - window_start_ms) <= 40) {
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

    // Anti-Drop Retry: If 15 ms has elapsed without a response, retransmit once
    if (!retried && (millis() - window_start_ms) >= 15) {
      retried = true;
      set_station_tx_mode();
      send_packet_raw(&challenge);
      set_station_rx_mode();
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

    Serial.print(F("  >>> [INTRUDER ALERT] No Valid Badge Responded in 40 ms Window! (Elapsed: "));
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
  Serial.println(F("Channel : 76 (2.476 GHz) | Rate: 1 Mbps | Timeout: 40 ms"));
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
  _delay_ms(1000); // 1-second challenge interval (simulating periodic sector sweep)
}
