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
#define CRSF_UART_BAUDRATE    CRSF_BAUDRATE     // 416666 bps
#define CRSF_TX_PIN           0                 // GP0 (Output to Flight Controller RX)
#define CRSF_RX_PIN           1                 // GP1
#define LED_PIN               PICO_DEFAULT_LED_PIN // GP25

// LoRa RF Parameters for 100Hz
#define LORA_BANDWIDTH        2                 // 2: 500 kHz
#define LORA_SPREADING_FACTOR 5                 // SF5 (Allows full 100Hz with 5.46ms ToA)
#define LORA_CODINGRATE       1                 // 1: 4/5
#define LORA_PREAMBLE_LENGTH  12                // >= 12 for SF5/SF6
#define LORA_FIX_LEN          false             // Variable length
#define LORA_CRC_ON           false             // CRSF has internal CRC8

// FHSS States
typedef enum {
    FHSS_STATE_SEARCHING = 0, // Park on Ch 0 listening for TX sync
    FHSS_STATE_LOCKED    = 1  // Synchronized and hopping with flywheel timer
} fhss_rx_state_t;

// Radio Callbacks & State
static RadioEvents_t RadioEvents;

static volatile fhss_rx_state_t fhss_state = FHSS_STATE_SEARCHING;
static volatile uint8_t current_rx_hop_idx = FHSS_SYNC_CHANNEL_IDX;
static volatile uint64_t next_hop_time_us = 0;
static volatile uint64_t last_packet_time_us = 0;
static volatile uint32_t rx_packet_count = 0;
static volatile uint32_t fhss_hop_count = 0;
static volatile int16_t last_rssi = 0;
static volatile int8_t  last_snr = 0;

static void OnTxDone( void )
{
    Radio.Standby( );
}

static void OnTxTimeout( void )
{
    Radio.Standby( );
}

static void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Standby( );

    uint64_t now_us = time_us_64( );

    // Air packet structure: [Byte 0: FHSS Header] + [Byte 1..size-1: CRSF Frame]
    if( size >= 5 && size <= ( CRSF_MAX_FRAME_SIZE + 1 ) )
    {
        uint8_t fhss_hdr = payload[0];
        uint8_t pkt_hop_idx = 0;

        if( fhss_validate_header( fhss_hdr, &pkt_hop_idx ) )
        {
            uint8_t *crsf_frame = &payload[1];
            uint16_t crsf_size = size - 1;
            uint8_t frame_len = crsf_frame[1];

            if( ( frame_len + 2 ) == crsf_size )
            {
                uint8_t calc_crc = crsf_crc8( &crsf_frame[2], crsf_size - 3 );
                if( calc_crc == crsf_frame[crsf_size - 1] )
                {
                    last_packet_time_us = now_us;
                    last_rssi = rssi;
                    last_snr = snr;
                    rx_packet_count++;

                    // Ensure target address is Flight Controller (0xC8) so FC doesn't discard
                    if( crsf_frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED )
                    {
                        crsf_frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
                    }

                    // Verified CRSF frame! Output directly to Flight Controller UART
                    uart_write_blocking( CRSF_UART, crsf_frame, crsf_size );

                    // Immediately switch to the next channel for the next 100Hz slot!
                    fhss_state = FHSS_STATE_LOCKED;
                    current_rx_hop_idx = ( pkt_hop_idx + 1 ) % FHSS_NUM_CHANNELS;
                    // Next packet expected in 10ms. Flywheel deadline set to 12.5ms
                    next_hop_time_us = now_us + 12500;

                    Radio.SetChannel( fhss_get_freq( current_rx_hop_idx ) );
                    SX126xAntSwOn( );
                    Radio.Rx( 0 );
                    return;
                }
            }
        }
    }

    // Unrecognized or corrupted frame - stay on current channel
    SX126xAntSwOn( );
    Radio.Rx( 0 );
}

static void OnRxTimeout( void )
{
    Radio.Standby( );
    SX126xClearDeviceErrors( );
    SX126xAntSwOn( );
    Radio.Rx( 0 );
}

static void OnRxError( void )
{
    Radio.Standby( );
    SX126xClearDeviceErrors( );
    SX126xAntSwOn( );
    Radio.Rx( 0 );
}

int main( void )
{
    stdio_init_all( );

    BoardInitMcu( );
    BoardInitPeriph( );

    gpio_init( LED_PIN );
    gpio_set_dir( LED_PIN, GPIO_OUT );
    gpio_put( LED_PIN, 0 );

    // Initialize UART0 for CRSF output to Flight Controller
    uart_init( CRSF_UART, CRSF_UART_BAUDRATE );
    gpio_set_function( CRSF_TX_PIN, GPIO_FUNC_UART );
    gpio_set_function( CRSF_RX_PIN, GPIO_FUNC_UART );
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
    Radio.SetChannel( fhss_get_freq( FHSS_SYNC_CHANNEL_IDX ) );

    Radio.SetRxConfig( MODEM_LORA,
                       LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE,
                       0,
                       LORA_PREAMBLE_LENGTH,
                       0,
                       LORA_FIX_LEN,
                       CRSF_MAX_FRAME_SIZE + 1,
                       LORA_CRC_ON,
                       0,
                       0,
                       false,
                       true ); // Continuous RX

    SX126xAntSwOn( );
    Radio.Rx( 0 ); // Start continuous receiving on Sync Channel 0

    printf( "\r\n=========================================\r\n" );
    printf( "   RP2040-LoRa CRSF RX Receiver (FHSS)   \r\n" );
    printf( "   FHSS: 8 Channels (431.0 ~ 435.0 MHz)  \r\n" );
    printf( "   UART Output: 416666 bps on GP0 (TX)   \r\n" );
    printf( "=========================================\r\n" );

    uint64_t last_stat_time_us = time_us_64( );

    while( 1 )
    {
        // 1. Process Radio IRQ (dispatches OnRxDone)
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }

        uint64_t now_us = time_us_64( );

        // 2. FHSS Hopping Engine & Flywheel
        if( fhss_state == FHSS_STATE_LOCKED )
        {
            // If the expected packet didn't arrive by deadline, flywheel hops to next channel!
            if( now_us >= next_hop_time_us )
            {
                current_rx_hop_idx = ( current_rx_hop_idx + 1 ) % FHSS_NUM_CHANNELS;
                next_hop_time_us += 10000; // Next hop in 10ms
                fhss_hop_count++;

                Radio.Standby( );
                Radio.SetChannel( fhss_get_freq( current_rx_hop_idx ) );
                SX126xAntSwOn( );
                Radio.Rx( 0 );
            }

            // If no packet received for > 300ms, unlock and return to search on Ch 0
            if( ( now_us - last_packet_time_us ) > 300000 )
            {
                fhss_state = FHSS_STATE_SEARCHING;
                current_rx_hop_idx = FHSS_SYNC_CHANNEL_IDX;
                Radio.Standby( );
                Radio.SetChannel( fhss_get_freq( FHSS_SYNC_CHANNEL_IDX ) );
                SX126xAntSwOn( );
                Radio.Rx( 0 );
            }
        }

        // 3. Failsafe check (if no packet for > 150ms)
        bool failsafe = ( ( now_us - last_packet_time_us ) > 150000 );

        // 4. LED Status indication
        if( failsafe )
        {
            // Blink LED slowly (2Hz) on Failsafe
            gpio_put( LED_PIN, ( ( now_us / 250000 ) % 2 ) );
        }
        else
        {
            // Solid ON when receiving healthy stream
            gpio_put( LED_PIN, 1 );
        }

        // 5. Output Status Statistics every 1 second
        if( ( now_us - last_stat_time_us ) >= 1000000 )
        {
            last_stat_time_us = now_us;
            uint32_t cur_freq = fhss_get_freq( current_rx_hop_idx );
            printf( "[RX] Rate: %lu pkts/s | FHSS: %s (Ch %u %lu.%01luMHz) | RSSI: -%ddBm | SNR: %ddB\r\n",
                    (unsigned long)rx_packet_count,
                    ( fhss_state == FHSS_STATE_LOCKED ) ? "LOCKED 100Hz" : "SEARCHING (Ch0)",
                    current_rx_hop_idx,
                    (unsigned long)( cur_freq / 1000000 ),
                    (unsigned long)( ( cur_freq % 1000000 ) / 100000 ),
                    -last_rssi / 2,
                    last_snr / 4 );
            rx_packet_count = 0;
            fhss_hop_count = 0;
        }
    }
}
