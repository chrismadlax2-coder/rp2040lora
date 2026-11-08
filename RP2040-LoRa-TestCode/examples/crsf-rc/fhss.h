#ifndef __FHSS_H__
#define __FHSS_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/unique_id.h"
#include "crsf.h"

// Hardware Pin Definition for Pairing / Binding (Connect GP2 to GND to trigger Bind Mode)
#define BIND_PIN                 2

// Number of frequency channels in the hopping sequence
#define FHSS_NUM_CHANNELS        8

// Base Sync / Bind Channel (Where RX parks during cold search or binding)
#define FHSS_SYNC_CHANNEL_IDX    0

// 8 Hopping Frequencies across 431.0 MHz ~ 435.0 MHz (Non-consecutive interleaving)
// Adjacent hops differ by >= 1.5 MHz for maximum resistance to narrowband jamming.
static const uint32_t FHSS_HOP_TABLE[FHSS_NUM_CHANNELS] = {
    431500000, // Ch 0: 431.5 MHz (Default Sync / Binding / Acquisition channel)
    433500000, // Ch 1: 433.5 MHz
    432000000, // Ch 2: 432.0 MHz
    434500000, // Ch 3: 434.5 MHz
    432500000, // Ch 4: 432.5 MHz
    435000000, // Ch 5: 435.0 MHz
    431000000, // Ch 6: 431.0 MHz
    433000000  // Ch 7: 433.0 MHz
};

// Air Packet Types & Header Definitions
#define FHSS_PKT_TYPE_BIND       0xBD   // Magic byte for Pairing Beacon
#define FHSS_HEADER_SIZE_RC      5      // 1 byte hop_idx + 4 bytes tx_uid
#define FHSS_PACKET_SIZE_BIND    7      // 0xBD (1B) + tx_uid (4B) + salt (1B) + crc8 (1B)

// Get 32-bit unique hardware identifier from RP2040 Flash ID
static inline uint32_t fhss_get_board_uid(void)
{
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    // Extract lower 4 bytes as 32-bit UID (unique per board)
    uint32_t uid = (uint32_t)board_id.id[4] |
                   ((uint32_t)board_id.id[5] << 8) |
                   ((uint32_t)board_id.id[6] << 16) |
                   ((uint32_t)board_id.id[7] << 24);
    return uid;
}

// Build 5-byte RC air packet header: [hop_idx (1B)] [tx_uid (4B, LE)]
static inline void fhss_build_rc_header(uint8_t *dest, uint8_t hop_idx, uint32_t tx_uid)
{
    dest[0] = (hop_idx & 0x0F);
    dest[1] = (uint8_t)(tx_uid & 0xFF);
    dest[2] = (uint8_t)((tx_uid >> 8) & 0xFF);
    dest[3] = (uint8_t)((tx_uid >> 16) & 0xFF);
    dest[4] = (uint8_t)((tx_uid >> 24) & 0xFF);
}

// Build 7-byte Bind beacon packet: [0xBD] [tx_uid (4B)] [salt (1B)] [crc8 (1B)]
static inline void fhss_build_bind_packet(uint8_t *dest, uint32_t tx_uid, uint8_t salt)
{
    dest[0] = FHSS_PKT_TYPE_BIND;
    dest[1] = (uint8_t)(tx_uid & 0xFF);
    dest[2] = (uint8_t)((tx_uid >> 8) & 0xFF);
    dest[3] = (uint8_t)((tx_uid >> 16) & 0xFF);
    dest[4] = (uint8_t)((tx_uid >> 24) & 0xFF);
    dest[5] = salt;
    dest[6] = crsf_crc8(dest, 6);
}

// Parse received air packet
// Returns:
//   0: Invalid or mismatched packet (should be dropped)
//   1: Valid RC packet matching bound_tx_uid
//   2: Valid BIND beacon packet
static inline int fhss_parse_packet(const uint8_t *buf, uint16_t size, uint32_t bound_uid, 
                                   uint8_t *out_hop_idx, uint32_t *out_sender_uid)
{
    if (buf == NULL || size == 0) return 0;

    // Check for BIND Beacon frame (7 bytes, starts with 0xBD)
    if (size == FHSS_PACKET_SIZE_BIND && buf[0] == FHSS_PKT_TYPE_BIND) {
        if (crsf_crc8(buf, 6) == buf[6]) {
            if (out_sender_uid != NULL) {
                *out_sender_uid = (uint32_t)buf[1] |
                                  ((uint32_t)buf[2] << 8) |
                                  ((uint32_t)buf[3] << 16) |
                                  ((uint32_t)buf[4] << 24);
            }
            return 2; // BIND Beacon
        }
        return 0; // Invalid BIND CRC
    }

    // Check for Standard RC frame (At least 5 bytes header + CRSF minimum frame)
    if (size >= (FHSS_HEADER_SIZE_RC + CRSF_RC_FRAME_SIZE)) {
        uint32_t pkt_uid = (uint32_t)buf[1] |
                           ((uint32_t)buf[2] << 8) |
                           ((uint32_t)buf[3] << 16) |
                           ((uint32_t)buf[4] << 24);
        if (out_sender_uid != NULL) {
            *out_sender_uid = pkt_uid;
        }

        // Only accept if UID matches our bound TX UID
        if (pkt_uid == bound_uid) {
            if (out_hop_idx != NULL) {
                *out_hop_idx = (buf[0] & 0x0F);
            }
            return 1; // Valid matching RC packet
        }
        return 0; // UID mismatch (another TX in the air)
    }

    return 0;
}

// Get frequency for a given hop index
static inline uint32_t fhss_get_freq(uint8_t hop_idx)
{
    return FHSS_HOP_TABLE[hop_idx % FHSS_NUM_CHANNELS];
}

#endif // __FHSS_H__
