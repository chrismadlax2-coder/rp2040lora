#ifndef __FHSS_H__
#define __FHSS_H__

#include <stdint.h>
#include <stdbool.h>

// Number of frequency channels in the hopping sequence
#define FHSS_NUM_CHANNELS        8

// Base Sync Channel (Where RX parks during cold search)
#define FHSS_SYNC_CHANNEL_IDX    0

// 8 Hopping Frequencies across 431.0 MHz ~ 435.0 MHz (Non-consecutive interleaving)
// Adjacent hops differ by >= 1.5 MHz for maximum resistance to narrowband jamming.
static const uint32_t FHSS_HOP_TABLE[FHSS_NUM_CHANNELS] = {
    431500000, // Ch 0: 431.5 MHz (Default Sync / Acquisition channel)
    433500000, // Ch 1: 433.5 MHz
    432000000, // Ch 2: 432.0 MHz
    434500000, // Ch 3: 434.5 MHz
    432500000, // Ch 4: 432.5 MHz
    435000000, // Ch 5: 435.0 MHz
    431000000, // Ch 6: 431.0 MHz
    433000000  // Ch 7: 433.0 MHz
};

// FHSS Air Packet Magic UID (High 4 bits) - prevents cross-talk between different TXs
#define FHSS_MAGIC_UID           0xA0
#define FHSS_UID_MASK            0xF0
#define FHSS_HOP_INDEX_MASK      0x0F

// Build 1-byte FHSS header containing Network UID and current Hop Index
static inline uint8_t fhss_build_header(uint8_t hop_idx)
{
    return (uint8_t)(FHSS_MAGIC_UID | (hop_idx & FHSS_HOP_INDEX_MASK));
}

// Validate 1-byte FHSS header and extract hop index
static inline bool fhss_validate_header(uint8_t header, uint8_t *out_hop_idx)
{
    if ((header & FHSS_UID_MASK) == FHSS_MAGIC_UID) {
        if (out_hop_idx != NULL) {
            *out_hop_idx = (header & FHSS_HOP_INDEX_MASK);
        }
        return true;
    }
    return false;
}

// Get frequency for a given hop index
static inline uint32_t fhss_get_freq(uint8_t hop_idx)
{
    return FHSS_HOP_TABLE[hop_idx % FHSS_NUM_CHANNELS];
}

#endif // __FHSS_H__
