#ifndef __CRSF_H__
#define __CRSF_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CRSF_BAUDRATE                     416666
#define CRSF_MAX_FRAME_SIZE               64
#define CRSF_RC_FRAME_SIZE                26

// Channel Standard Values
#define CRSF_CHANNEL_VALUE_MIN            172   // ~988 us (Minimum stick / Throttle cutoff / Switch low)
#define CRSF_CHANNEL_VALUE_MID            992   // 1500 us (Center stick)
#define CRSF_CHANNEL_VALUE_MAX            1811  // ~2012 us (Maximum stick / Switch high)

// Device Addresses
#define CRSF_ADDRESS_BROADCAST            0x00
#define CRSF_ADDRESS_FLIGHT_CONTROLLER    0xC8  // Flight Controller
#define CRSF_ADDRESS_HANDSET              0xEA  // Radio Transmitter Handset
#define CRSF_ADDRESS_RECEIVER             0xEC  // CRSF Receiver
#define CRSF_ADDRESS_TX_MODULE            0xEE  // CRSF Transmitter Module

// Frame Types
#define CRSF_FRAMETYPE_GPS                0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR     0x08
#define CRSF_FRAMETYPE_LINK_STATISTICS    0x14
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_FRAMETYPE_ATTITUDE           0x1E
#define CRSF_FRAMETYPE_FLIGHT_MODE        0x21

// CRSF Channels (11 bits per channel, 16 channels packed into 22 bytes)
typedef struct {
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11;
    unsigned int ch6 : 11;
    unsigned int ch7 : 11;
    unsigned int ch8 : 11;
    unsigned int ch9 : 11;
    unsigned int ch10 : 11;
    unsigned int ch11 : 11;
    unsigned int ch12 : 11;
    unsigned int ch13 : 11;
    unsigned int ch14 : 11;
    unsigned int ch15 : 11;
} __attribute__((__packed__)) crsf_channels_t;

// Standard CRSF CRC8 calculation (DVB-S2 polynomial 0xD5)
static inline uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Build a standard Failsafe frame for Flight Controller
// (Throttle=MIN, Roll/Pitch/Yaw=MID, AUX1/ARM=MIN, AUX2..12=MID)
static inline void crsf_build_failsafe_frame( uint8_t *out_frame )
{
    out_frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // 0xC8
    out_frame[1] = 24; // Type(1) + Payload(22) + CRC(1) = 24
    out_frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    crsf_channels_t channels;
    channels.ch0 = CRSF_CHANNEL_VALUE_MID; // Roll: 1500us (Center)
    channels.ch1 = CRSF_CHANNEL_VALUE_MID; // Pitch: 1500us (Center)
    channels.ch2 = CRSF_CHANNEL_VALUE_MIN; // Throttle: 988us (Minimum / Cutoff)
    channels.ch3 = CRSF_CHANNEL_VALUE_MID; // Yaw: 1500us (Center)
    channels.ch4 = CRSF_CHANNEL_VALUE_MIN; // AUX1 (ARM switch): 988us (Disarmed)
    channels.ch5 = CRSF_CHANNEL_VALUE_MIN; // AUX2: 988us
    channels.ch6 = CRSF_CHANNEL_VALUE_MIN; // AUX3: 988us
    channels.ch7 = CRSF_CHANNEL_VALUE_MIN; // AUX4: 988us
    channels.ch8 = CRSF_CHANNEL_VALUE_MID; // AUX5..12: 1500us
    channels.ch9 = CRSF_CHANNEL_VALUE_MID;
    channels.ch10 = CRSF_CHANNEL_VALUE_MID;
    channels.ch11 = CRSF_CHANNEL_VALUE_MID;
    channels.ch12 = CRSF_CHANNEL_VALUE_MID;
    channels.ch13 = CRSF_CHANNEL_VALUE_MID;
    channels.ch14 = CRSF_CHANNEL_VALUE_MID;
    channels.ch15 = CRSF_CHANNEL_VALUE_MID;

    memcpy( &out_frame[3], &channels, sizeof(channels) );
    out_frame[25] = crsf_crc8( &out_frame[2], 23 );
}

// CRSF Parser State Machine
typedef enum {
    CRSF_PARSER_SYNC,
    CRSF_PARSER_LEN,
    CRSF_PARSER_DATA
} crsf_parser_state_t;

typedef struct {
    crsf_parser_state_t state;
    uint8_t buffer[CRSF_MAX_FRAME_SIZE];
    uint8_t length;
    uint8_t index;
} crsf_parser_t;

static inline void crsf_parser_init(crsf_parser_t *parser)
{
    parser->state = CRSF_PARSER_SYNC;
    parser->length = 0;
    parser->index = 0;
}

// Returns true when a complete valid frame is parsed
static inline bool crsf_parse_byte(crsf_parser_t *parser, uint8_t b)
{
    switch (parser->state) {
        case CRSF_PARSER_SYNC:
            // Allow any standard CRSF sync address (Module 0xEE, Radio 0xEA, Receiver 0xEC, FC 0xC8, Broadcast 0x00)
            if (b == CRSF_ADDRESS_TX_MODULE || b == CRSF_ADDRESS_HANDSET || 
                b == CRSF_ADDRESS_RECEIVER || b == CRSF_ADDRESS_FLIGHT_CONTROLLER || 
                b == CRSF_ADDRESS_BROADCAST) {
                parser->buffer[0] = b;
                parser->state = CRSF_PARSER_LEN;
            }
            break;

        case CRSF_PARSER_LEN:
            // Frame length includes: Type + Payload + CRC (min 2, max 62)
            if (b >= 2 && b <= (CRSF_MAX_FRAME_SIZE - 2)) {
                parser->buffer[1] = b;
                parser->length = b + 2; // Total frame length including Addr and Len
                parser->index = 2;
                parser->state = CRSF_PARSER_DATA;
            } else {
                parser->state = CRSF_PARSER_SYNC;
            }
            break;

        case CRSF_PARSER_DATA:
            parser->buffer[parser->index++] = b;
            if (parser->index >= parser->length) {
                parser->state = CRSF_PARSER_SYNC;
                // Validate CRC (calculated over Type + Payload, buffer[2] to buffer[length - 2])
                uint8_t calculated_crc = crsf_crc8(&parser->buffer[2], parser->length - 3);
                uint8_t received_crc = parser->buffer[parser->length - 1];
                if (calculated_crc == received_crc) {
                    return true;
                }
            }
            break;
    }
    return false;
}

#endif // __CRSF_H__
