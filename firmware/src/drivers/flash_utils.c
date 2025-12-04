/*
 * flash_utils.c - Flash Memory Utilities
 *
 * Shared flash helper functions for NVM operations.
 * Extracted from valve_nvm.c and network_nvm.c to eliminate duplication.
 */

#include "drivers/flash_utils.h"
#include "config/nvm.h"
#include "stm32h7xx_hal.h"

uint32_t
flash_get_bank(uint32_t address)
{
    const uint32_t bank_split = FLASH_BASE + FLASH_BANK_SIZE;
    return (address < bank_split) ? FLASH_BANK_1 : FLASH_BANK_2;
}

uint32_t
flash_get_sector(uint32_t address)
{
    const uint32_t bank_base = (flash_get_bank(address) == FLASH_BANK_1) ?
        FLASH_BASE : (FLASH_BASE + FLASH_BANK_SIZE);
    return (address - bank_base) / NVM_FLASH_SECTOR_SIZE;
}

status_t
flash_erase_sector(uint32_t address)
{
    HAL_FLASH_Unlock();

    const uint32_t bank = flash_get_bank(address);
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = bank,
        .Sector = flash_get_sector(address),
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3  /* 2.7-3.6V */
    };

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

status_t
flash_program_data(uint32_t address, const void *data, size_t size)
{
    if (data == NULL || size == 0U) {
        return STATUS_ERROR_INVALID_PARAM;
    }

    /* STM32H7 requires 32-byte aligned programming */
    if ((address % 32U) != 0U || (size % 32U) != 0U) {
        return STATUS_ERROR_INVALID_PARAM;
    }

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;
    const uint32_t *src = (const uint32_t *)data;
    uint32_t addr = address;

    /* Program in 32-byte (256-bit) flash words */
    for (size_t i = 0; i < size; i += 32U) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)src);
        if (status != HAL_OK) {
            break;
        }
        src += 8U;  /* 8 * 4 bytes = 32 bytes */
        addr += 32U;
    }

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

uint32_t
flash_calculate_checksum(const void *data, size_t size)
{
    if (data == NULL || size == 0U) {
        return 0U;
    }

    uint32_t checksum = 0U;
    const uint8_t *ptr = (const uint8_t *)data;

    for (size_t i = 0; i < size; i++) {
        checksum ^= ptr[i];
    }

    return checksum;
}
