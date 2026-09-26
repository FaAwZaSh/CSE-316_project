#ifndef VANGUARD_RF_PROTOCOL_H
#define VANGUARD_RF_PROTOCOL_H

#include <stdint.h>

// =====================================================================================
//  PROJECT VANGUARD: Autonomous Sentry & Wearable IFF Challenge-Response Protocol
// =====================================================================================
//  RF Frequency:      2.476 GHz (Channel 76)
//  RF Data Rate:      1 Mbps
//  RF Power:          -12 dBm (0x02 - High SNR, anti-saturation)
//  Payload Width:     32 Bytes (Fixed)
// =====================================================================================

#define VANGUARD_RF_CHANNEL       76
#define VANGUARD_RF_SETUP_VAL     0x27   // 250 kbps, 0 dBm (Max Power), High LNA Gain
#define VANGUARD_SECRET_KEY       0x5A3C9E17UL

#define PACKET_TYPE_CHALLENGE     0x01
#define PACKET_TYPE_RESPONSE      0x02

// 5-Byte Shared Over-The-Air Address
static const uint8_t VANGUARD_RF_ADDR[5] = {'V', 'A', 'N', 'G', '1'};

// ---------- 32-Byte Challenge Packet (Station -> Badge) ----------
struct __attribute__((packed)) IFFChallengePacket {
  uint8_t  packet_type;     // PACKET_TYPE_CHALLENGE (0x01)
  uint8_t  station_id;      // Station ID (0x01 = Main Turret)
  uint16_t session_seq;     // Rolling sequence counter
  uint32_t nonce;           // Cryptographic Challenge Nonce
  uint32_t station_uptime;  // Station timestamp (ms)
  char     magic[8];        // "VANGUARD" header
  uint8_t  reserved[12];    // Padding to exactly 32 bytes
};

// ---------- 32-Byte Keyed Response Packet (Badge -> Station) ----------
struct __attribute__((packed)) IFFResponsePacket {
  uint8_t  packet_type;     // PACKET_TYPE_RESPONSE (0x02)
  uint8_t  badge_id;        // Wearable Badge ID (1, 2, 3...)
  uint16_t session_seq;     // Echoed sequence counter
  uint32_t nonce_echo;      // Echoed Nonce (Replay protection)
  uint32_t auth_token;      // Keyed Auth: nonce ^ VANGUARD_SECRET_KEY
  uint32_t badge_uptime;    // Badge timestamp (ms)
  char     badge_callsign[8]; // e.g. "VIP_01  "
  uint8_t  status_flags;    // Battery / Emergency flags
  uint8_t  reserved[7];     // Padding to exactly 32 bytes
};

// Verify struct sizes at compile time
static_assert(sizeof(IFFChallengePacket) == 32, "IFFChallengePacket must be 32 bytes!");
static_assert(sizeof(IFFResponsePacket) == 32, "IFFResponsePacket must be 32 bytes!");

#endif // VANGUARD_RF_PROTOCOL_H
