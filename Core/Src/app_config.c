#include "app_config.h"

#include <stddef.h>
#include <string.h>

/*
 * Trang Flash cuối của STM32F103C8 64 KB:
 *
 * 0x0800FC00 ... 0x0800FFFF
 */
#define APP_CONFIG_FLASH_ADDRESS \
    0x0800FC00UL

static AppConfig_t g_app_config;

static uint32_t AppConfig_CalculateCRC32(
    const uint8_t *data,
    uint32_t length)
{
    uint32_t crc;
    uint32_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    crc = 0xFFFFFFFFUL;

    for (index = 0U;
         index < length;
         index++)
    {
        crc ^= data[index];

        for (bit = 0U;
             bit < 8U;
             bit++)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc =
                    (crc >> 1U) ^
                    0xEDB88320UL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint8_t AppConfig_IsFrequencyValid(
    uint32_t frequency)
{
    switch (frequency)
    {
        case 433000000UL:
        case 470000000UL:
        case 868000000UL:
        case 915000000UL:
        case 920000000UL:
            return 1U;

        default:
            return 0U;
    }
}

void AppConfig_SetDefaults(void)
{
    uint8_t index;

    memset(
        &g_app_config,
        0,
        sizeof(g_app_config));

    g_app_config.magic =
        APP_CONFIG_MAGIC;

    g_app_config.version =
        APP_CONFIG_VERSION;

    g_app_config.size =
        sizeof(AppConfig_t);

    g_app_config.node_id = 1U;
    g_app_config.destination_id = 2U;
    g_app_config.role = APP_ROLE_TX;

    g_app_config.frequency =
        920000000UL;

    g_app_config.bandwidth = 0U;
    g_app_config.spreading_factor = 10U;
    g_app_config.coding_rate = 1U;
    g_app_config.tx_power = 14;

    /*
     * Khởi tạo tất cả slot về trạng thái tắt.
     */
    for (index = 0U;
         index < APP_MAX_SENSORS;
         index++)
    {
        g_app_config
            .sensors[index]
            .enabled = 0U;

        g_app_config
            .sensors[index]
            .slave_id =
            (uint8_t)(index + 1U);

        g_app_config
            .sensors[index]
            .function_code = 3U;

        g_app_config
            .sensors[index]
            .start_register =
            0x0000U;

        g_app_config
            .sensors[index]
            .register_count = 1U;
    }

    /*
     * Mặc định ba cảm biến cũ.
     */
    g_app_config.sensors[0].enabled = 1U;
    g_app_config.sensors[0].slave_id = 1U;
    g_app_config.sensors[0].register_count = 1U;

    g_app_config.sensors[1].enabled = 1U;
    g_app_config.sensors[1].slave_id = 2U;
    g_app_config.sensors[1].register_count = 1U;

    g_app_config.sensors[2].enabled = 1U;
    g_app_config.sensors[2].slave_id = 3U;
    g_app_config.sensors[2].register_count = 7U;
}

uint8_t AppConfig_Validate(
    const AppConfig_t *config)
{
    uint8_t index;
    uint8_t other_index;

    uint32_t calculated_crc;

    if (config == NULL)
    {
        return 0U;
    }

    if (config->magic !=
        APP_CONFIG_MAGIC)
    {
        return 0U;
    }

    if (config->version !=
        APP_CONFIG_VERSION)
    {
        return 0U;
    }

    if (config->size !=
        sizeof(AppConfig_t))
    {
        return 0U;
    }

    calculated_crc =
        AppConfig_CalculateCRC32(
            (const uint8_t *)config,
            offsetof(
                AppConfig_t,
                crc32));

    if (calculated_crc !=
        config->crc32)
    {
        return 0U;
    }

    if (config->role >
        APP_ROLE_RX)
    {
        return 0U;
    }

    if (AppConfig_IsFrequencyValid(
            config->frequency) == 0U)
    {
        return 0U;
    }

    if (config->bandwidth > 2U)
    {
        return 0U;
    }

    if ((config->spreading_factor < 7U) ||
        (config->spreading_factor > 12U))
    {
        return 0U;
    }

    if ((config->coding_rate < 1U) ||
        (config->coding_rate > 4U))
    {
        return 0U;
    }

    if ((config->tx_power < -9) ||
        (config->tx_power > 22))
    {
        return 0U;
    }

    for (index = 0U;
         index < APP_MAX_SENSORS;
         index++)
    {
        const AppSensorConfig_t *sensor =
            &config->sensors[index];

        if (sensor->enabled > 1U)
        {
            return 0U;
        }

        if (sensor->enabled == 0U)
        {
            continue;
        }

        if ((sensor->slave_id < 1U) ||
            (sensor->slave_id >
             APP_MAX_SENSORS))
        {
            return 0U;
        }

        if ((sensor->function_code != 3U) &&
            (sensor->function_code != 4U))
        {
            return 0U;
        }

        if ((sensor->register_count < 1U) ||
            (sensor->register_count >
             APP_MAX_REGISTERS))
        {
            return 0U;
        }

        for (other_index =
                 (uint8_t)(index + 1U);
             other_index <
                 APP_MAX_SENSORS;
             other_index++)
        {
            const AppSensorConfig_t *other =
                &config->sensors[
                    other_index];

            if ((other->enabled != 0U) &&
                (other->slave_id ==
                 sensor->slave_id))
            {
                return 0U;
            }
        }
    }

    return 1U;
}

void AppConfig_Init(void)
{
    AppConfig_t flash_config;

    memcpy(
        &flash_config,
        (const void *)
            APP_CONFIG_FLASH_ADDRESS,
        sizeof(flash_config));

    if (AppConfig_Validate(
            &flash_config) != 0U)
    {
        memcpy(
            &g_app_config,
            &flash_config,
            sizeof(g_app_config));
    }
    else
    {
        AppConfig_SetDefaults();
    }
}

AppConfig_t *AppConfig_Get(void)
{
    return &g_app_config;
}

HAL_StatusTypeDef AppConfig_Save(void)
{
    FLASH_EraseInitTypeDef erase;
    HAL_StatusTypeDef status;

    uint32_t page_error;
    uint32_t offset;
    uint32_t address;

    uint16_t halfword;

    g_app_config.magic =
        APP_CONFIG_MAGIC;

    g_app_config.version =
        APP_CONFIG_VERSION;

    g_app_config.size =
        sizeof(AppConfig_t);

    g_app_config.crc32 =
        AppConfig_CalculateCRC32(
            (const uint8_t *)
                &g_app_config,
            offsetof(
                AppConfig_t,
                crc32));

    if (AppConfig_Validate(
            &g_app_config) == 0U)
    {
        return HAL_ERROR;
    }

    status = HAL_FLASH_Unlock();

    if (status != HAL_OK)
    {
        return status;
    }

    memset(
        &erase,
        0,
        sizeof(erase));

    erase.TypeErase =
        FLASH_TYPEERASE_PAGES;

    erase.PageAddress =
        APP_CONFIG_FLASH_ADDRESS;

    erase.NbPages = 1U;

    page_error = 0U;

    status =
        HAL_FLASHEx_Erase(
            &erase,
            &page_error);

    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    address =
        APP_CONFIG_FLASH_ADDRESS;

    for (offset = 0U;
         offset < sizeof(AppConfig_t);
         offset += 2U)
    {
        halfword = 0xFFFFU;

        memcpy(
            &halfword,
            ((const uint8_t *)
                &g_app_config) +
                offset,
            sizeof(halfword));

        status =
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_HALFWORD,
                address,
                halfword);

        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }

        address += 2U;
    }

    HAL_FLASH_Lock();

    return HAL_OK;
}
