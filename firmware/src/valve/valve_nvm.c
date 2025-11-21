/*
 * valve_nvm.c
 *
 * Non-volatile memory management for valve presets using STM32H7 flash.
 * Stores user-editable presets persistently across reboots.
 */

#include "valve_nvm.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/* NVM data structure */
#define NVM_VERSION 2  /* Increment when struct preset_params layout changes */
#define NVM_PRESET_COUNT 4

struct nvm_header {
    uint32_t version;
    uint32_t checksum;
    uint32_t write_count;  /* For wear leveling tracking */
};

struct nvm_data {
    struct nvm_header header;
    struct preset_params presets[NVM_PRESET_COUNT];
};

static const struct preset_params default_presets[NVM_PRESET_COUNT] = {
    /* VALVE_PRESET_LIGHT */
    {
        .name = "Light",
        .torque_limit_nm = 8.0f,
        .default_travel_deg = 90.0f,
        .hil_b_viscous_nm_s_per_rad = 0.02f,
        .hil_tau_c_coulomb_nm = 0.06f,
        .hil_k_w_wall_stiffness_nm_per_turn = 10.0f,
        .hil_c_w_wall_damping_nm_s_per_turn = 0.1f,
        .hil_eps_smoothing = 1e-3f
    },
    /* VALVE_PRESET_MEDIUM */
    {
        .name = "Medium",
        .torque_limit_nm = 8.0f,
        .default_travel_deg = 90.0f,
        .hil_b_viscous_nm_s_per_rad = 0.04f,
        .hil_tau_c_coulomb_nm = 0.12f,
        .hil_k_w_wall_stiffness_nm_per_turn = 15.0f,
        .hil_c_w_wall_damping_nm_s_per_turn = 0.2f,
        .hil_eps_smoothing = 1e-3f
    },
    /* VALVE_PRESET_HEAVY */
    {
        .name = "Heavy",
        .torque_limit_nm = 6.0f,
        .default_travel_deg = 360.0f,
        .hil_b_viscous_nm_s_per_rad = 0.08f,
        .hil_tau_c_coulomb_nm = 0.25f,
        .hil_k_w_wall_stiffness_nm_per_turn = 25.0f,
        .hil_c_w_wall_damping_nm_s_per_turn = 0.4f,
        .hil_eps_smoothing = 1e-3f
    },
    /* VALVE_PRESET_INDUSTRIAL */
    {
        .name = "Industrial",
        .torque_limit_nm = 8.0f,
        .default_travel_deg = 360.0f,
        .hil_b_viscous_nm_s_per_rad = 0.12f,
        .hil_tau_c_coulomb_nm = 0.40f,
        .hil_k_w_wall_stiffness_nm_per_turn = 35.0f,
        .hil_c_w_wall_damping_nm_s_per_turn = 0.6f,
        .hil_eps_smoothing = 1e-3f
    }
};

/* Flash address for NVM data */
extern uint8_t __nvm_data_start[];
#define NVM_FLASH_ADDR ((uint32_t)&__nvm_data_start[0])
#define NVM_DATA_SIZE sizeof(struct nvm_data)

/* CRC calculation for validation */
static uint32_t calculate_checksum(const struct nvm_data *data) {
    /* Simple XOR checksum for now; could use HAL_CRC if available */
    uint32_t checksum = 0;
    const uint8_t *ptr = (const uint8_t *)data;
    for (size_t i = 0; i < sizeof(struct nvm_data); i++) {
        checksum ^= ptr[i];
    }
    return checksum;
}

static void set_checksum(struct nvm_data *data) {
    data->header.checksum = 0U;
    data->header.checksum = calculate_checksum(data);
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
static status_t erase_nvm_sector(void) {
    HAL_FLASH_Unlock();

    const uint32_t bank = get_bank(NVM_FLASH_ADDR);
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = bank,
        .Sector = get_sector(NVM_FLASH_ADDR),
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3  /* 2.7-3.6V */
    };

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

/* Program NVM data to flash */
static status_t program_nvm_data(const struct nvm_data *data) {
    HAL_FLASH_Unlock();

    HAL_StatusTypeDef status = HAL_OK;
    const uint32_t *src = (const uint32_t *)data;  /* 256-bit aligned */
    uint32_t addr = NVM_FLASH_ADDR;

    for (size_t i = 0; i < NVM_DATA_SIZE; i += 32) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)src);
        if (status != HAL_OK) break;
        src += 8;  /* 8 * 4 bytes = 32 */
        addr += 32;
    }

    HAL_FLASH_Lock();

    return (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
}

/* Load NVM data from flash */
static status_t load_nvm_data(struct nvm_data *data) {
    memcpy(data, (const void *)NVM_FLASH_ADDR, NVM_DATA_SIZE);

    /* Validate version and checksum */
    if (data->header.version != NVM_VERSION) {
        return STATUS_ERROR;
    }

    uint32_t stored_checksum = data->header.checksum;
    data->header.checksum = 0U;
    uint32_t calculated_checksum = calculate_checksum(data);
    data->header.checksum = stored_checksum;

    if (calculated_checksum != stored_checksum) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/* Initialize NVM with default presets if invalid */
status_t valve_nvm_init(void) {
    __attribute__((aligned(32))) struct nvm_data data;

    if (load_nvm_data(&data) == STATUS_OK) {
        /* Data is valid, no init needed */
        return STATUS_OK;
    }

    /* Initialize with default presets */
    data.header.version = NVM_VERSION;
    data.header.write_count = 0;

    memcpy(data.presets, default_presets, sizeof(default_presets));

    /* Calculate checksum */
    set_checksum(&data);

    /* Erase and program */
    if (erase_nvm_sector() != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (program_nvm_data(&data) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/* Load presets from NVM */
status_t valve_nvm_load_presets(struct preset_params presets_out[NVM_PRESET_COUNT]) {
    struct nvm_data data;

    if (load_nvm_data(&data) != STATUS_OK) {
        return STATUS_ERROR;
    }

    memcpy(presets_out, data.presets, sizeof(data.presets));
    return STATUS_OK;
}

const struct preset_params *valve_nvm_get_default_presets(void)
{
    return default_presets;
}

/* Save presets to NVM */
status_t valve_nvm_save_presets(const struct preset_params presets_in[NVM_PRESET_COUNT]) {
    __attribute__((aligned(32))) struct nvm_data data;

    /* Load current data to preserve header */
    if (load_nvm_data(&data) != STATUS_OK) {
        /* If load fails, reinitialize header */
        data.header.version = NVM_VERSION;
        data.header.write_count = 0;
    }

    data.header.write_count++;

    memcpy(data.presets, presets_in, sizeof(data.presets));

    /* Recalculate checksum */
    set_checksum(&data);

    /* Erase and program */
    if (erase_nvm_sector() != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (program_nvm_data(&data) != STATUS_OK) {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}
