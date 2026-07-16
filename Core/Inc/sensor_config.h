#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "main.h"
#include <stdint.h>

#define SENSOR_CONFIG_MAGIC                 0x53454E53UL
#define SENSOR_CONFIG_VERSION               0x0001U

#define SENSOR_CONFIG_MAX_SENSORS           8U
#define SENSOR_CONFIG_MAX_REGISTERS         16U
#define SENSOR_PACKET_MAX_SIZE              255U

/*
 * Cấu hình một cảm biến Modbus.
 */
typedef struct
{
    uint8_t enabled;
    uint8_t slave_id;
    uint8_t function_code;
    uint8_t register_count;

    uint16_t start_register;
    uint16_t reserved;
} SensorItemConfig_t;

/*
 * Toàn bộ cấu hình node.
 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t structure_size;

    uint8_t node_id;
    uint8_t retry_count;
    uint16_t sensor_gap_ms;

    uint32_t poll_period_ms;

    SensorItemConfig_t sensors[SENSOR_CONFIG_MAX_SENSORS];

    uint32_t crc32;
} SensorNodeConfig_t;

void SensorConfig_SetDefaults(
    SensorNodeConfig_t *config
);

uint8_t SensorConfig_Load(
    SensorNodeConfig_t *config
);

uint8_t SensorConfig_Save(
    SensorNodeConfig_t *config
);

uint8_t SensorConfig_IsValid(
    const SensorNodeConfig_t *config
);

uint8_t SensorConfig_SetSensor(
    SensorNodeConfig_t *config,
    uint8_t index,
    const SensorItemConfig_t *item
);

uint16_t SensorConfig_CalculatePacketSize(
    const SensorNodeConfig_t *config
);

uint32_t SensorConfig_GetFlashAddress(void);

#endif
