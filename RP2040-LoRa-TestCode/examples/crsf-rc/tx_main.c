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
static uint32_t my_tx_uid = 0;

static void OnTxDone( void )
{
    Radio.Standby( );
    tx_busy = false;
}

static void OnTxTimeout( void )
{
    Radio.Standby( );
    tx_busy = false;
    SX126xClearDeviceErrors( );
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

    // 2. Wrap CRSF data with 5-byte header: [hop_idx (1B)] [my_tx_uid (4B)]
    static uint8_t air_packet[FHSS_HEADER_SIZE_RC + CRSF_MAX_FRAME_SIZE];
    fhss_build_rc_header( air_packet, current_hop_idx, my_tx_uid );
    memcpy( &air_packet[FHSS_HEADER_SIZE_RC], data, size );

    Radio.Send( air_packet, size + FHSS_HEADER_SIZE_RC );

    // 3. Advance to next hop index
    current_hop_idx = ( current_hop_idx + 1 ) % FHSS_NUM_CHANNELS;
    return true;
}

static bool lora_transmit_bind_beacon( void )
{
    if( tx_busy )
    {
        return false;
    }
    tx_busy = true;

    // Send Bind Beacon on Ch 0 (431.5 MHz)
    Radio.Standby( );
    Radio.SetChannel( fhss_get_freq( FHSS_SYNC_CHANNEL_IDX ) );
    SX126xAntSwOff( );

    static uint8_t bind_packet[FHSS_PACKET_SIZE_BIND];
    fhss_build_bind_packet( bind_packet, my_tx_uid, 0 );

    Radio.Send( bind_packet, FHSS_PACKET_SIZE_BIND );
    return true;
}

int main( void )
{
    stdio_init_all( );

    BoardInitMcu( );
    BoardInitPeriph( );

    gpio_init( LED_PIN );
    gpio_set_dir( LED_PIN, GPIO_OUT );
    gpio_put( LED_PIN, 0 );

    // Initialize GP2 as BIND trigger pin with internal pull-up
    gpio_init( BIND_PIN );
    gpio_set_dir( BIND_PIN, GPIO_IN );
    gpio_pull_up( BIND_PIN );

    // Read unique hardware UID from RP2040 Flash ID
    my_tx_uid = fhss_get_board_uid( );

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
    printf( "   TX Board UID: 0x%08lX\r\n", (unsigned long)my_tx_uid );
    printf( "   GP2: BIND Pin (Connect to GND to Pair)\r\n" );
    printf( "   FHSS: 8 Channels (431.0 ~ 435.0 MHz)  \r\n" );
    printf( "   BW: 500kHz, SF5, Power: 22dBm         \r\n" );
    printf( "   UART: 400000 bps on GP1 (RX Inverted) \r\n" );
    printf( "=========================================\r\n" );

    crsf_parser_t parser;
    crsf_parser_init( &parser );

    uint32_t last_stat_time = to_ms_since_boot( get_absolute_time( ) );
    uint64_t last_tx_time_us = time_us_64( );
    uint64_t last_bind_beacon_us = 0;
    uint32_t last_real_rx_time = 0;
    uint32_t packet_counter = 0;
    uint32_t uart_rx_byte_count = 0;
    uint32_t uart_valid_frame_count = 0;
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

        uint32_t now = to_ms_since_boot( get_absolute_time( ) );
        uint64_t now_us = time_us_64( );

        // 2. Check GP2 BIND trigger pin (LOW = Grounded = BIND MODE)
        bool is_bind_mode = ( gpio_get( BIND_PIN ) == 0 );

        if( is_bind_mode )
        {
            // --- BIND BEACON MODE (Broadcasts on Ch 0 every 50ms) ---
            if( ( now_us - last_bind_beacon_us ) >= 50000 )
            {
                last_bind_beacon_us = now_us;
                lora_transmit_bind_beacon( );
                gpio_xor_mask( 1ul << LED_PIN ); // Rapid blink (10Hz)
            }

            if( ( now - last_stat_time ) >= 1000 )
            {
                last_stat_time = now;
                printf( "[TX BIND] Broadcasting Pair Beacon | UID: 0x%08lX | Ch 0 (431.5MHz)\r\n", 
                        (unsigned long)my_tx_uid );
            }
            continue; // Skip normal RC transmission while binding
        }

        // --- NORMAL OPERATION ---

        // 3. Read incoming bytes from Handset UART and update latest frame cache
        while( uart_is_readable( CRSF_UART ) )
        {
            uint8_t c = uart_getc( CRSF_UART );
            uart_rx_byte_count++;
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

        // 4. Hardware-timed 100 Hz (10,000 us) RF Transmission Engine
        if( ( now_us - last_tx_time_us ) >= 10000 )
        {
            last_tx_time_us += 10000;
            if( ( now_us - last_tx_time_us ) > 20000 )
            {
                last_tx_time_us = now_us; // Resync if paused
            }

            if( using_real_handset && ( now - last_real_rx_time ) <= 100 && latest_crsf_len > 0 )
            {
                // Transmit latest handset stick positions at exact 100Hz
                if( lora_transmit( latest_crsf_frame, latest_crsf_len ) )
                {
                    packet_counter++;
                }
            }
            else
            {
                // Fallback to safe failsafe neutral frame (Throttle=MIN, Roll/Pitch/Yaw=MID, ARM=MIN)
                using_real_handset = false;
                uint8_t sim_frame[CRSF_RC_FRAME_SIZE];
                crsf_build_failsafe_frame( sim_frame );
                if( lora_transmit( sim_frame, CRSF_RC_FRAME_SIZE ) )
                {
                    packet_counter++;
                }
            }
        }

        // 5. Output Status Statistics every 1 second
        if( ( now - last_stat_time ) >= 1000 )
        {
            last_stat_time = now;
            uint32_t cur_freq = fhss_get_freq( current_hop_idx );
            printf( "[TX] UID: 0x%08lX | Rate: %lu pkts/s | FHSS: HOPPING (Ch %u %lu.%01luMHz) | Mode: %s | GP1: %lu B/s (Valid: %lu)\r\n",
                    (unsigned long)my_tx_uid,
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
            gpio_xor_mask( 1ul << LED_PIN ); // Slow blink indicator
        }
    }
}
