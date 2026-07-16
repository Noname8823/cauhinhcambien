#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

#define APP_CONFIG_MAGIC       0xA55A1234UL
#define APP_CONFIG_VERSION     1U

#define APP_MAX_SENSORS        8U
#define APP_MAX_REGISTERS      7U

#define APP_ROLE_TX            0U
#define APP_ROLE_RX            1U

typedef struct
{
    uint8_t enabled;
    uint8_t slave_id;
    uint8_t function_code;
    uint8_t register_count;

    uint16_t start_register;
    uint16_t reserved;
} AppSensorConfig_t;

typedef struct
{
    uint32_t magic;

    uint16_t version;
    uint16_t size;

    uint8_t node_id;
    uint8_t destination_id;
    uint8_t role;
    uint8_t bandwidth;

    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power;
    uint8_t reserved0;

    uint32_t frequency;

    AppSensorConfig_t sensors[
        APP_MAX_SENSORS];

    uint32_t crc32;
} AppConfig_t;

void AppConfig_Init(void);

void AppConfig_SetDefaults(void);

AppConfig_t *AppConfig_Get(void);

uint8_t AppConfig_Validate(
    const AppConfig_t *config
);

HAL_StatusTypeDef AppConfig_Save(void);

#endif
