#ifndef __NVM_CONFIG_H__
#define __NVM_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

#define NVM_MAGIC               0x4C524332 // "LRC2" (LoRa RC 2)
// Use the very last 4KB sector of 2MB SPI Flash: (2048 * 1024 - 4096) = 0x1FF000
#define NVM_FLASH_OFFSET        (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define NVM_FLASH_ADDR          ((const uint8_t *)(XIP_BASE + NVM_FLASH_OFFSET))

typedef struct {
    uint32_t magic;
    uint32_t bound_tx_uid;
    uint8_t  fhss_salt;
    uint8_t  is_bound;
    uint8_t  reserved[6];
    uint32_t checksum;
} __attribute__((packed)) rx_config_t;

// Calculate simple checksum
static inline uint32_t nvm_checksum(const rx_config_t *cfg)
{
    return (cfg->magic ^ cfg->bound_tx_uid ^ ((uint32_t)cfg->fhss_salt << 8) ^ ((uint32_t)cfg->is_bound << 16));
}

// Load configuration from Flash. Returns true if valid config found.
static inline bool nvm_load_config(rx_config_t *cfg)
{
    const rx_config_t *flash_cfg = (const rx_config_t *)NVM_FLASH_ADDR;
    if (flash_cfg->magic == NVM_MAGIC && flash_cfg->checksum == nvm_checksum(flash_cfg)) {
        memcpy(cfg, flash_cfg, sizeof(rx_config_t));
        return true;
    }
    // Uninitialized or invalid
    memset(cfg, 0, sizeof(rx_config_t));
    return false;
}

// Save configuration to Flash. Erases the sector and programs one 256-byte page.
static inline bool nvm_save_config(const rx_config_t *cfg)
{
    uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, sizeof(page_buf));

    rx_config_t to_write = *cfg;
    to_write.magic = NVM_MAGIC;
    to_write.checksum = nvm_checksum(&to_write);

    memcpy(page_buf, &to_write, sizeof(rx_config_t));

    // Disable interrupts during Flash erase and program
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(NVM_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(NVM_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    // Verify written data
    const rx_config_t *verify_cfg = (const rx_config_t *)NVM_FLASH_ADDR;
    return (verify_cfg->magic == NVM_MAGIC && 
            verify_cfg->bound_tx_uid == cfg->bound_tx_uid &&
            verify_cfg->checksum == to_write.checksum);
}

// Erase configuration to force un-bound state
static inline void nvm_clear_config(void)
{
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(NVM_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

#endif // __NVM_CONFIG_H__
