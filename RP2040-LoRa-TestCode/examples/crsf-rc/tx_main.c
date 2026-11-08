#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "board.h"
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "crsf.h"
#include "fhss.h"

// Hardware Configuration
#define CRSF_UART             uart0
#define CRSF_UART_BAUDRATE    400000            // 400,000 bps (Exact OpenTX / EdgeTX standard)
#define CRSF_TX_PIN           0                 // GP0
#define CRSF_RX_PIN           1                 // GP1 (Handset DATA input)
#define LED_PIN               PICO_DEFAULT_LED_PIN // GP25

// LoRa RF Parameters for 100Hz
#define TX_OUTPUT_POWER       22                // dBm
#define LORA_BANDWIDTH        2                 // 2: 500 kHz
#define LORA_SPREADING_FACTOR 5                 // SF5 (Allows full 100Hz with 5.46ms ToA)
#define LORA_CODINGRATE       1                 // 1: 4/5
#define LORA_PREAMBLE_LENGTH  12                // >= 12 for SF5/SF6
#define LORA_FIX_LEN          false             // Variable length
#define LORA_CRC_ON           false             // CRSF has internal CRC8

// Radio Callbacks & State
static RadioEvents_t RadioEvents;
static volatile bool tx_busy = false;
static uint8_t current_hop_idx = 0;

static void OnTxDone( void )
{
    Radio.Standby( );
    tx_busy = false;
}

static void OnTxTimeout( void )
{
    Radio.Standby( );
    tx_busy = false;
    SX126xClearDeviceErrors();
}

static void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );
}

static void OnRxTimeout( void )
{
    Radio.Standby( );
}

static void OnRxError( void )
{
    Radio.Standby( );
}

static bool lora_transmit( const uint8_t *data, uint8_t size )
{
    if( tx_busy )
    {
        return false; // Previous packet still in air, skip to avoid queue latency
    }
    tx_busy = true;

    // 1. Hop to next RF frequency
    uint32_t freq = fhss_get_freq( current_hop_idx );
    Radio.Standby( );
    Radio.SetChannel( freq );
    SX126xAntSwOff( ); // Switch RF antenna switch to TX mode

    // 2. Wrap CRSF data with 1-byte FHSS header (contains UID and hop_idx)
    static uint8_t air_packet[CRSF_MAX_FRAME_SIZE + 1];
    air_packet[0] = fhss_build_header( current_hop_idx );
    memcpy( &air_packet[1], data, size );

    Radio.Send( air_packet, size + 1 );

    // 3. Advance to next hop index
    current_hop_idx = ( current_hop_idx + 1 ) % FHSS_NUM_CHANNELS;
    return true;
}

// Build a simulated CRSF RC Channels frame (for standalone testing without handset)
static void crsf_build_simulated_frame( uint8_t *out_frame, uint16_t sweep_val )
{
    out_frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // 0xC8 (Target address for Flight Controller)
    out_frame[1] = 24; // Length: Type (1) + Payload (22) + CRC (1) = 24
    out_frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    crsf_channels_t channels;
    // Ch0 (Roll): dynamic sweep between 600 and 1400
    channels.ch0 = sweep_val;
    // Ch1 (Pitch), Ch2 (Throttle), Ch3 (Yaw): 992 (1500us standard center)
    channels.ch1 = 992;
    channels.ch2 = 992;
    channels.ch3 = 992;
    // Aux 1..12: 992
    channels.ch4 = 992;
    channels.ch5 = 992;
    channels.ch6 = 992;
    channels.ch7 = 992;
    channels.ch8 = 992;
    channels.ch9 = 992;
    channels.ch10 = 992;
    channels.ch11 = 992;
    channels.ch12 = 992;
    channels.ch13 = 992;
    channels.ch14 = 992;
    channels.ch15 = 992;

    memcpy( &out_frame[3], &channels, sizeof(channels) );
    out_frame[25] = crsf_crc8( &out_frame[2], 23 ); // CRC over Type (1) + Payload (22)
}

int main( void )
{
    stdio_init_all( );

    BoardInitMcu( );
    BoardInitPeriph( );

    gpio_init( LED_PIN );
    gpio_set_dir( LED_PIN, GPIO_OUT );
    gpio_put( LED_PIN, 0 );

    // Initialize UART0 for CRSF input from Handset
    uart_init( CRSF_UART, CRSF_UART_BAUDRATE );
    gpio_set_function( CRSF_TX_PIN, GPIO_FUNC_UART );
    gpio_set_function( CRSF_RX_PIN, GPIO_FUNC_UART );
    gpio_set_inover( CRSF_RX_PIN, GPIO_OVERRIDE_INVERT ); // Invert input for OpenTX inverted UART
    uart_set_hw_flow( CRSF_UART, false, false );
    uart_set_format( CRSF_UART, 8, 1, UART_PARITY_NONE );
    uart_set_fifo_enabled( CRSF_UART, true );

    // Initialize Radio
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( fhss_get_freq( 0 ) );

    Radio.SetTxConfig( MODEM_LORA,
                       TX_OUTPUT_POWER,
                       0,
                       LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH,
                       LORA_FIX_LEN,
                       LORA_CRC_ON,
                       0,
                       0,
                       false,
                       3000 );

    SX126xAntSwOff( );

    printf( "\r\n=========================================\r\n" );
    printf( "   RP2040-LoRa CRSF TX Module (FHSS 100Hz)\r\n" );
    printf( "   FHSS: 8 Channels (431.0 ~ 435.0 MHz)  \r\n" );
    printf( "   BW: 500kHz, SF5, Power: 22dBm         \r\n" );
    printf( "   UART: 400000 bps on GP1 (RX Inverted) \r\n" );
    printf( "=========================================\r\n" );

    crsf_parser_t parser;
    crsf_parser_init( &parser );

    uint32_t last_stat_time = to_ms_since_boot( get_absolute_time( ) );
    uint64_t last_tx_time_us = time_us_64( );
    uint32_t last_real_rx_time = 0;
    uint32_t packet_counter = 0;
    uint32_t uart_rx_byte_count = 0;
    uint32_t uart_valid_frame_count = 0;
    uint8_t raw_sample[64] = {0};
    uint8_t raw_sample_idx = 0;
    uint16_t sweep_val = 600;
    int16_t sweep_dir = 10;
    bool using_real_handset = false;

    // Cache for the latest valid CRSF frame from handset
    static uint8_t latest_crsf_frame[CRSF_MAX_FRAME_SIZE];
    static uint8_t latest_crsf_len = 0;

    while( 1 )
    {
        // 1. Process Radio IRQ (dispatches OnTxDone)
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }

        // 2. Read incoming bytes from Handset UART and update latest frame cache
        while( uart_is_readable( CRSF_UART ) )
        {
            uint8_t c = uart_getc( CRSF_UART );
            uart_rx_byte_count++;
            if( raw_sample_idx < 64 )
            {
                raw_sample[raw_sample_idx++] = c;
            }
            if( crsf_parse_byte( &parser, c ) )
            {
                // Full valid CRSF frame received from handset!
                uart_valid_frame_count++;
                memcpy( latest_crsf_frame, parser.buffer, parser.length );
                latest_crsf_len = parser.length;
                last_real_rx_time = to_ms_since_boot( get_absolute_time( ) );
                using_real_handset = true;
            }
        }

        uint32_t now = to_ms_since_boot( get_absolute_time( ) );
        uint64_t now_us = time_us_64( );

        // 3. Hardware-timed 100 Hz (10,000 us) RF Transmission Engine
        // Decoupled from handset UART rate (OpenTX may send at 250Hz; RF strictly hops at 100Hz)
        if( ( now_us - last_tx_time_us ) >= 10000 )
        {
            last_tx_time_us += 10000;
            if( ( now_us - last_tx_time_us ) > 20000 )
            {
                last_tx_time_us = now_us; // Resync if paused
            }

            if( using_real_handset && ( now - last_real_rx_time ) <= 1000 && latest_crsf_len > 0 )
            {
                // Transmit latest handset stick positions at exact 100Hz
                if( lora_transmit( latest_crsf_frame, latest_crsf_len ) )
                {
                    packet_counter++;
                }
            }
            else
            {
                // Fallback to simulated test generator (100Hz sweep) if no handset signal
                using_real_handset = false;
                uint8_t sim_frame[CRSF_RC_FRAME_SIZE];
                crsf_build_simulated_frame( sim_frame, sweep_val );
                if( lora_transmit( sim_frame, CRSF_RC_FRAME_SIZE ) )
                {
                    packet_counter++;
                }

                sweep_val += sweep_dir;
                if( sweep_val >= 1400 ) { sweep_dir = -10; }
                else if( sweep_val <= 600 ) { sweep_dir = 10; }
            }
        }

        // 4. Output Status Statistics every 1 second
        if( ( now - last_stat_time ) >= 1000 )
        {
            last_stat_time = now;
            uint32_t cur_freq = fhss_get_freq( current_hop_idx );
            printf( "[TX] Rate: %lu pkts/s | FHSS: HOPPING (Ch %u %lu.%01luMHz) | Mode: %s | GP1: %lu B/s (Valid: %lu)\r\n",
                    (unsigned long)packet_counter,
                    current_hop_idx,
                    (unsigned long)(cur_freq / 1000000),
                    (unsigned long)((cur_freq % 1000000) / 100000),
                    using_real_handset ? "HANDSET UART" : "SIMULATED TEST (100Hz)",
                    (unsigned long)uart_rx_byte_count,
                    (unsigned long)uart_valid_frame_count );
            packet_counter = 0;
            uart_rx_byte_count = 0;
            uart_valid_frame_count = 0;
            raw_sample_idx = 0;
            gpio_xor_mask( 1ul << LED_PIN ); // Toggle LED
        }
    }
}
