#include "network_nvm.h"
#include "stm32h7xx_hal.h"
#include <string.h>

// CRC32 polynomial (standard)
#define CRC32_POLY 0xEDB88320

uint32_t network_nvm_calculate_checksum(const struct network_nvm_config *config) {
    if (config == NULL) return 0;
    
    uint32_t checksum = 0;
    const uint8_t *ptr = (const uint8_t *)config;
    size_t len = sizeof(*config) - sizeof(config->checksum);  // Exclude checksum field

    for (size_t i = 0; i < len; i++) {
        checksum ^= ptr[i];
    }
    return checksum;
}

/* Flash helper utilities */
static uint32_t get_bank(uint32_t address)
{
    const uint32_t bank_split = FLASH_BASE + FLASH_BANK_SIZE;
    return (address < bank_split) ? FLASH_BANK_1 : FLASH_BANK_2;
}

static uint32_t get_sector(uint32_t address)
{
    const uint32_t bank_base = (get_bank(address) == FLASH_BANK_1) ?
        FLASH_BASE : (FLASH_BASE + FLASH_BANK_SIZE);
    return (address - bank_base) / 0x20000U;  /* 128KB sectors within bank */
}

/* Erase NVM sector */
static bool erase_nvm_sector(void) {
    HAL_FLASH_Unlock();

    const uint32_t bank = get_bank(NETWORK_CONFIG_ADDR);
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = bank,
        .Sector = get_sector(NETWORK_CONFIG_ADDR),
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3  /* 2.7-3.6V */
    };

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK);
}

/* Program network config to flash */
static bool program_network_config(const struct network_nvm_config *config) {
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;
    const uint32_t *src = (const uint32_t *)config;  /* 256-bit aligned */
    uint32_t addr = NETWORK_CONFIG_ADDR;

    // Program 3 flashwords (96 bytes to cover 76-byte struct + padding)
    for (size_t i = 0; i < 96; i += 32) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)src);
        if (status != HAL_OK) break;
        src += 8;  /* 8 * 4 bytes = 32 */
        addr += 32;
    }

    HAL_FLASH_Lock();

    return (status == HAL_OK);
}

bool network_nvm_save(const struct network_nvm_config *config) {
    if (config == NULL) return false;

    // Prepare config with checksum
    __attribute__((aligned(32))) uint8_t buffer[96] = {0};  // 3 * 32 bytes
    struct network_nvm_config *config_to_save = (struct network_nvm_config *)buffer;
    *config_to_save = *config;
    config_to_save->checksum = network_nvm_calculate_checksum(config_to_save);

    // Erase sector
    if (!erase_nvm_sector()) {
        return false;
    }

    // Program data
    if (!program_network_config(config_to_save)) {
        return false;
    }

    return true;
}

bool network_nvm_load(struct network_nvm_config *config) {
    if (config == NULL) return false;
    
    // Read from flash - simple memcpy, no complex operations at boot
    const struct network_nvm_config *flash_config = (const struct network_nvm_config *)NETWORK_CONFIG_ADDR;
    
    // Quick validation before copying
    if (flash_config->magic != NETWORK_NVM_MAGIC) {
        return false;
    }
    
    // Copy data
    memcpy(config, flash_config, sizeof(*config));
    
    // Ensure strings are null-terminated
    config->ip_addr[sizeof(config->ip_addr) - 1] = '\0';
    config->netmask[sizeof(config->netmask) - 1] = '\0';
    config->gateway[sizeof(config->gateway) - 1] = '\0';
    
    // Validate version and checksum
    if (config->version != NETWORK_NVM_VERSION) {
        return false;
    }
    
    uint32_t expected_checksum = network_nvm_calculate_checksum(config);
    if (expected_checksum != config->checksum) {
        return false;
    }

    return true;
}
