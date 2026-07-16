#include "sensor_config.h"

#include <stddef.h>
#include <string.h>

/*
 * STM32F103C8 có page Flash 1 KB.
 * Cấu hình được lưu tại page cuối cùng.
 */
#define SENSOR_CONFIG_FLASH_PAGE_SIZE       1024UL

#define SENSOR_CONFIG_MIN_PERIOD_MS         200UL
#define SENSOR_CONFIG_MAX_PERIOD_MS         3600000UL

#define SENSOR_CONFIG_MAX_GAP_MS            1000U
#define SENSOR_CONFIG_MAX_RETRY             5U

/*
 * Kích thước cố định của packet:
 *
 * Header           2 byte
 * Length           1 byte
 * Node ID          1 byte
 * Sequence         1 byte
 * Digital input    1 byte
 * Record count     1 byte
 * CRC              2 byte
 *
 * Tổng             9 byte
 */
#define SENSOR_PACKET_FIXED_SIZE            9U

/*
 * Mỗi record:
 *
 * Slave ID         1 byte
 * Function code    1 byte
 * Status           1 byte
 * Register count   1 byte
 * Data             count * 2 byte
 */
#define SENSOR_PACKET_RECORD_HEADER_SIZE    4U

static uint32_t SensorConfig_CalculateCRC32(
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

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];

        for (bit = 0U; bit < 8U; bit++)
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

static uint8_t SensorConfig_ItemIsValid(
    const SensorItemConfig_t *item)
{
    if (item == NULL)
    {
        return 0U;
    }

    if (item->enabled > 1U)
    {
        return 0U;
    }

    /*
     * Khi sensor bị disable vẫn yêu cầu
     * dữ liệu cấu hình có dạng hợp lệ.
     */
    if ((item->slave_id == 0U) ||
        (item->slave_id > 247U))
    {
        return 0U;
    }

    if ((item->function_code != 0x03U) &&
        (item->function_code != 0x04U))
    {
        return 0U;
    }

    if ((item->register_count == 0U) ||
        (item->register_count >
         SENSOR_CONFIG_MAX_REGISTERS))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t SensorConfig_FieldsAreValid(
    const SensorNodeConfig_t *config)
{
    uint8_t sensor_index;

    if (config == NULL)
    {
        return 0U;
    }

    if ((config->poll_period_ms <
         SENSOR_CONFIG_MIN_PERIOD_MS) ||
        (config->poll_period_ms >
         SENSOR_CONFIG_MAX_PERIOD_MS))
    {
        return 0U;
    }

    if (config->sensor_gap_ms >
        SENSOR_CONFIG_MAX_GAP_MS)
    {
        return 0U;
    }

    if (config->retry_count >
        SENSOR_CONFIG_MAX_RETRY)
    {
        return 0U;
    }

    for (sensor_index = 0U;
         sensor_index < SENSOR_CONFIG_MAX_SENSORS;
         sensor_index++)
    {
        if (SensorConfig_ItemIsValid(
                &config->sensors[sensor_index]) == 0U)
        {
            return 0U;
        }
    }

    if (SensorConfig_CalculatePacketSize(config) >
        SENSOR_PACKET_MAX_SIZE)
    {
        return 0U;
    }

    return 1U;
}

static void SensorConfig_UpdateHeaderAndCRC(
    SensorNodeConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->magic = SENSOR_CONFIG_MAGIC;
    config->version = SENSOR_CONFIG_VERSION;
    config->structure_size =
        (uint16_t)sizeof(SensorNodeConfig_t);

    config->crc32 =
        SensorConfig_CalculateCRC32(
            (const uint8_t *)config,
            (uint32_t)offsetof(
                SensorNodeConfig_t,
                crc32));
}

uint32_t SensorConfig_GetFlashAddress(void)
{
    uint32_t flash_size_kb;

    /*
     * Thanh ghi hệ thống chứa kích thước Flash.
     */
    flash_size_kb =
        (uint32_t)(*(const uint16_t *)FLASHSIZE_BASE);

    /*
     * Một số chip clone trả về giá trị không đúng.
     * Dùng mặc định 64 KB nếu kết quả bất thường.
     */
    if ((flash_size_kb < 32UL) ||
        (flash_size_kb > 256UL))
    {
        flash_size_kb = 64UL;
    }

    return FLASH_BASE +
           (flash_size_kb * 1024UL) -
           SENSOR_CONFIG_FLASH_PAGE_SIZE;
}

void SensorConfig_SetDefaults(
    SensorNodeConfig_t *config)
{
    uint8_t sensor_index;

    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));

    config->node_id = 1U;
    config->retry_count = 2U;
    config->sensor_gap_ms = 20U;
    config->poll_period_ms = 1000UL;

    /*
     * Khởi tạo tất cả slot bằng giá trị hợp lệ,
     * nhưng mặc định disable.
     */
    for (sensor_index = 0U;
         sensor_index < SENSOR_CONFIG_MAX_SENSORS;
         sensor_index++)
    {
        config->sensors[sensor_index].enabled = 0U;
        config->sensors[sensor_index].slave_id =
            (uint8_t)(sensor_index + 1U);
        config->sensors[sensor_index].function_code = 0x03U;
        config->sensors[sensor_index].start_register = 0x0000U;
        config->sensors[sensor_index].register_count = 1U;
        config->sensors[sensor_index].reserved = 0U;
    }

    /*
     * Sensor 0: gió.
     */
    config->sensors[0].enabled = 1U;
    config->sensors[0].slave_id = 1U;
    config->sensors[0].function_code = 0x03U;
    config->sensors[0].start_register = 0x0000U;
    config->sensors[0].register_count = 1U;

    /*
     * Sensor 1: mưa.
     */
    config->sensors[1].enabled = 1U;
    config->sensors[1].slave_id = 2U;
    config->sensors[1].function_code = 0x03U;
    config->sensors[1].start_register = 0x0000U;
    config->sensors[1].register_count = 1U;

    /*
     * Sensor 2: đất.
     *
     * Mặc định chỉ đọc register nhiệt độ.
     * Muốn đọc đủ 7 thông số thì cấu hình count = 7.
     */
    config->sensors[2].enabled = 1U;
    config->sensors[2].slave_id = 3U;
    config->sensors[2].function_code = 0x03U;
    config->sensors[2].start_register = 0x0000U;
    config->sensors[2].register_count = 1U;

    SensorConfig_UpdateHeaderAndCRC(config);
}

uint16_t SensorConfig_CalculatePacketSize(
    const SensorNodeConfig_t *config)
{
    uint16_t packet_size;
    uint8_t sensor_index;
    const SensorItemConfig_t *item;

    if (config == NULL)
    {
        return 0U;
    }

    packet_size = SENSOR_PACKET_FIXED_SIZE;

    for (sensor_index = 0U;
         sensor_index < SENSOR_CONFIG_MAX_SENSORS;
         sensor_index++)
    {
        item = &config->sensors[sensor_index];

        if (item->enabled == 0U)
        {
            continue;
        }

        packet_size =
            (uint16_t)(
                packet_size +
                SENSOR_PACKET_RECORD_HEADER_SIZE +
                ((uint16_t)item->register_count * 2U));
    }

    return packet_size;
}

uint8_t SensorConfig_IsValid(
    const SensorNodeConfig_t *config)
{
    uint32_t calculated_crc;

    if (config == NULL)
    {
        return 0U;
    }

    if (config->magic != SENSOR_CONFIG_MAGIC)
    {
        return 0U;
    }

    if (config->version != SENSOR_CONFIG_VERSION)
    {
        return 0U;
    }

    if (config->structure_size !=
        sizeof(SensorNodeConfig_t))
    {
        return 0U;
    }

    if (SensorConfig_FieldsAreValid(config) == 0U)
    {
        return 0U;
    }

    calculated_crc =
        SensorConfig_CalculateCRC32(
            (const uint8_t *)config,
            (uint32_t)offsetof(
                SensorNodeConfig_t,
                crc32));

    if (calculated_crc != config->crc32)
    {
        return 0U;
    }

    return 1U;
}

uint8_t SensorConfig_Load(
    SensorNodeConfig_t *config)
{
    const SensorNodeConfig_t *flash_config;

    if (config == NULL)
    {
        return 0U;
    }

    flash_config =
        (const SensorNodeConfig_t *)
        SensorConfig_GetFlashAddress();

    memcpy(
        config,
        flash_config,
        sizeof(*config));

    if (SensorConfig_IsValid(config) == 0U)
    {
        SensorConfig_SetDefaults(config);
        return 0U;
    }

    return 1U;
}

uint8_t SensorConfig_Save(
    SensorNodeConfig_t *config)
{
    FLASH_EraseInitTypeDef erase_config;
    uint32_t page_error;
    uint32_t flash_address;
    uint32_t offset;
    uint16_t half_word;
    const uint8_t *source;

    if (config == NULL)
    {
        return 0U;
    }

    SensorConfig_UpdateHeaderAndCRC(config);

    if (SensorConfig_IsValid(config) == 0U)
    {
        return 0U;
    }

    flash_address =
        SensorConfig_GetFlashAddress();

    source = (const uint8_t *)config;

    HAL_FLASH_Unlock();

    memset(
        &erase_config,
        0,
        sizeof(erase_config));

    erase_config.TypeErase =
        FLASH_TYPEERASE_PAGES;

    erase_config.PageAddress =
        flash_address;

    erase_config.NbPages = 1U;

    page_error = 0U;

    if (HAL_FLASHEx_Erase(
            &erase_config,
            &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0U;
    }

    /*
     * STM32F103 ghi Flash theo half-word 16 bit.
     */
    for (offset = 0U;
         offset < sizeof(SensorNodeConfig_t);
         offset += 2U)
    {
        half_word = source[offset];

        if ((offset + 1U) <
            sizeof(SensorNodeConfig_t))
        {
            half_word |=
                (uint16_t)source[offset + 1U] << 8U;
        }
        else
        {
            half_word |= 0xFF00U;
        }

        if (HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_HALFWORD,
                flash_address + offset,
                half_word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0U;
        }
    }

    HAL_FLASH_Lock();

    return 1U;
}

uint8_t SensorConfig_SetSensor(
    SensorNodeConfig_t *config,
    uint8_t index,
    const SensorItemConfig_t *item)
{
    SensorItemConfig_t old_item;

    if ((config == NULL) ||
        (item == NULL) ||
        (index >= SENSOR_CONFIG_MAX_SENSORS))
    {
        return 0U;
    }

    if (SensorConfig_ItemIsValid(item) == 0U)
    {
        return 0U;
    }

    old_item = config->sensors[index];

    config->sensors[index] = *item;
    config->sensors[index].reserved = 0U;

    if (SensorConfig_FieldsAreValid(config) == 0U)
    {
        config->sensors[index] = old_item;
        return 0U;
    }

    return 1U;
}
