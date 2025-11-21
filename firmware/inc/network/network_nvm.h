#ifndef NETWORK_NVM_H
#define NETWORK_NVM_H

#include <stdint.h>
#include <stdbool.h>

#define NETWORK_NVM_MAGIC 0x4E455457  // "NETW"
#define NETWORK_NVM_VERSION 1

struct network_nvm_config {
    uint32_t magic;           // Magic number for validation
    uint32_t version;         // Version for future compatibility
    char ip_addr[16];         // IPv4 address string (e.g., "192.168.1.100")
    char netmask[16];         // Subnet mask string (e.g., "255.255.255.0")
    char gateway[16];         // Gateway string (e.g., "192.168.1.1")
    uint32_t checksum;        // CRC32 of data for integrity
};

/* Linker symbol for network config region */
extern uint8_t __network_config_start[];
#define NETWORK_CONFIG_ADDR ((uint32_t)&__network_config_start[0])

/* Function prototypes */
bool network_nvm_save(const struct network_nvm_config *config);
bool network_nvm_load(struct network_nvm_config *config);
uint32_t network_nvm_calculate_checksum(const struct network_nvm_config *config);

#endif /* NETWORK_NVM_H */
