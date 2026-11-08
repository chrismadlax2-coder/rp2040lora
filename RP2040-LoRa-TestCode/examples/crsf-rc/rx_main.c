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
#include "nvm_config.h"

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
    FHSS_STATE_BINDING   = 0, // Park on Ch 0 listening for TX Pair Beacon
    FHSS_STATE_SEARCHING = 1, // Park on Ch 0 listening for paired TX sync
    FHSS_STATE_LOCKED    = 2  // Synchronized and hopping with flywheel timer
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

static rx_config_t rx_cfg;
static volatile uint32_t bound_tx_uid = 0;
static volatile bool request_rebind = false;

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
    uint8_t pkt_hop_idx = 0;
    uint32_t sender_uid = 0;

    int parse_result = fhss_parse_packet( payload, size, bound_tx_uid, &pkt_hop_idx, &sender_uid );

    // 1. Process BIND Beacon Frame (0xBD)
    if( parse_result == 2 )
    {
        if( fhss_state == FHSS_STATE_BINDING || gpio_get( BIND_PIN ) == 0 || !rx_cfg.is_bound )
        {
            // Bind packet received while in bind mode!
            bound_tx_uid = sender_uid;
            rx_cfg.bound_tx_uid = sender_uid;
            rx_cfg.is_bound = 1;
            rx_cfg.fhss_salt = payload[5];

            // Save to Flash NVM
            nvm_save_config( &rx_cfg );

            printf( "\r\n[RX BIND] *** PAIRING SUCCESSFUL! Bound to TX UID: 0x%08lX ***\r\n", 
                    (unsigned long)sender_uid );

            // Exit bind mode if pin is released
            if( gpio_get( BIND_PIN ) != 0 )
            {
                fhss_state = FHSS_STATE_SEARCHING;
                current_rx_hop_idx = FHSS_SYNC_CHANNEL_IDX;
            }

            SX126xAntSwOn( );
            Radio.Rx( 0 );
            return;
        }
    }

    // 2. Process Standard RC Frame
    if( parse_result == 1 )
    {
        uint8_t *crsf_frame = &payload[FHSS_HEADER_SIZE_RC];
        uint16_t crsf_size = size - FHSS_HEADER_SIZE_RC;
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

                // Ensure target address is Flight Controller (0xC8)
                if( crsf_frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED )
                {
                    crsf_frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
                }

                // Output directly to Flight Controller UART
                uart_write_blocking( CRSF_UART, crsf_frame, crsf_size );

                // Advance immediately to next slot!
                fhss_state = FHSS_STATE_LOCKED;
                current_rx_hop_idx = ( pkt_hop_idx + 1 ) % FHSS_NUM_CHANNELS;
                next_hop_time_us = now_us + 12500;

                Radio.SetChannel( fhss_get_freq( current_rx_hop_idx ) );
                SX126xAntSwOn( );
                Radio.Rx( 0 );
                return;
            }
        }
    }

    // Unmatched packet or corrupted frame - continue listening on current channel
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

    // Initialize GP2 as BIND trigger pin with internal pull-up
    gpio_init( BIND_PIN );
    gpio_set_dir( BIND_PIN, GPIO_IN );
    gpio_pull_up( BIND_PIN );

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
                       FHSS_HEADER_SIZE_RC + CRSF_MAX_FRAME_SIZE,
                       LORA_CRC_ON,
                       0,
                       0,
                       false,
                       true ); // Continuous RX

    SX126xAntSwOn( );
    Radio.Rx( 0 ); // Start continuous receiving on Sync Channel 0

    // Load paired TX configuration from RP2040 Flash NVM
    bool has_config = nvm_load_config( &rx_cfg );
    if( has_config && rx_cfg.is_bound && rx_cfg.bound_tx_uid != 0 )
    {
        bound_tx_uid = rx_cfg.bound_tx_uid;
        fhss_state = FHSS_STATE_SEARCHING;
    }
    else
    {
        bound_tx_uid = 0;
        fhss_state = FHSS_STATE_BINDING;
    }

    // Check if GP2 is grounded on boot -> force bind mode
    if( gpio_get( BIND_PIN ) == 0 )
    {
        fhss_state = FHSS_STATE_BINDING;
    }

    printf( "\r\n=========================================\r\n" );
    printf( "   RP2040-LoRa CRSF RX Receiver (FHSS)   \r\n" );
    printf( "   GP2: BIND Pin (Connect to GND to Pair)\r\n" );
    if( rx_cfg.is_bound )
    {
        printf( "   Paired TX UID: 0x%08lX (from Flash)\r\n", (unsigned long)bound_tx_uid );
    }
    else
    {
        printf( "   Status: UNBOUND - Waiting for Pair\r\n" );
    }
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

        // 2. Check GP2 BIND trigger pin
        if( gpio_get( BIND_PIN ) == 0 )
        {
            if( fhss_state != FHSS_STATE_BINDING )
            {
                fhss_state = FHSS_STATE_BINDING;
                current_rx_hop_idx = FHSS_SYNC_CHANNEL_IDX;
                Radio.Standby( );
                Radio.SetChannel( fhss_get_freq( FHSS_SYNC_CHANNEL_IDX ) );
                SX126xAntSwOn( );
                Radio.Rx( 0 );
                printf( "[RX] GP2 Grounded -> Entering BIND MODE on 431.5MHz...\r\n" );
            }
        }

        // 3. Handle Binding Mode
        if( fhss_state == FHSS_STATE_BINDING )
        {
            // Rapid blink LED (10Hz)
            gpio_put( LED_PIN, ( ( now_us / 50000 ) % 2 ) );

            if( ( now_us - last_stat_time_us ) >= 1000000 )
            {
                last_stat_time_us = now_us;
                printf( "[RX BIND] Waiting for TX Beacon on Ch 0 (431.5MHz)... Current Bound: 0x%08lX\r\n",
                        (unsigned long)bound_tx_uid );
            }
            continue; // Do not run hopping engine while in bind mode
        }

        // 4. FHSS Hopping Engine & Flywheel
        if( fhss_state == FHSS_STATE_LOCKED )
        {
            // If expected packet didn't arrive by deadline, flywheel hops to next channel!
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

        // 5. Failsafe check (if no packet for > 100ms)
        bool failsafe = ( ( now_us - last_packet_time_us ) > 100000 );
        static uint64_t last_failsafe_out_time_us = 0;

        // 6. LED Status indication and Active Failsafe RC Frame Injection
        if( failsafe )
        {
            // Blink LED slowly (2Hz) on Failsafe
            gpio_put( LED_PIN, ( ( now_us / 250000 ) % 2 ) );

            // Actively output Failsafe CRSF frame at 50Hz (every 20ms) to Flight Controller
            if( ( now_us - last_failsafe_out_time_us ) >= 20000 )
            {
                last_failsafe_out_time_us = now_us;
                uint8_t fs_frame[CRSF_RC_FRAME_SIZE];
                crsf_build_failsafe_frame( fs_frame );
                uart_write_blocking( CRSF_UART, fs_frame, CRSF_RC_FRAME_SIZE );
            }
        }
        else
        {
            // Solid ON when receiving healthy stream from bound TX
            gpio_put( LED_PIN, 1 );
        }

        // 7. Output Status Statistics every 1 second
        if( ( now_us - last_stat_time_us ) >= 1000000 )
        {
            last_stat_time_us = now_us;
            uint32_t cur_freq = fhss_get_freq( current_rx_hop_idx );
            printf( "[RX] Rate: %lu pkts/s | FHSS: %s (Ch %u %lu.%01luMHz) | Bound UID: 0x%08lX | RSSI: -%ddBm | SNR: %ddB\r\n",
                    (unsigned long)rx_packet_count,
                    ( fhss_state == FHSS_STATE_LOCKED ) ? "LOCKED 100Hz" : "SEARCHING (Ch0)",
                    current_rx_hop_idx,
                    (unsigned long)( cur_freq / 1000000 ),
                    (unsigned long)( ( cur_freq % 1000000 ) / 100000 ),
                    (unsigned long)bound_tx_uid,
                    -last_rssi / 2,
                    last_snr / 4 );
            rx_packet_count = 0;
            fhss_hop_count = 0;
        }
    }
}
