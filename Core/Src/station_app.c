#include "station_app.h"

#include "app_config.h"
#include "at_command.h"
#include "modbus_master.h"
#include "rs485_port.h"

#include <string.h>

#define SENSOR_RETRY_COUNT     2U
#define SENSOR_GAP_MS          20U
#define SEND_PERIOD_MS         1000U

#define PACKET_HEADER_0        0xA5U
#define PACKET_HEADER_1        0x5AU

#define INPUT_ACTIVE_STATE     GPIO_PIN_SET

/*
 * Fixed:
 *
 * Header 2
 * Length 1
 * DI 1
 * CRC 2
 *
 * Sensor max:
 * SID 1 + count 1 + 14 byte data
 *
 * Tổng lớn nhất:
 * 6 + 8 × 16 = 134 byte.
 */
#define PACKET_MAX_SIZE        134U

typedef struct
{
    ModbusStatus_t status;

    uint16_t values[
        APP_MAX_REGISTERS];
} SensorRuntime_t;

/*
 * USART1:
 *
 * USB-RS485 và cảm biến cùng chung bus.
 */
static RS485_Port_t uart1_bus;

/*
 * USART2:
 *
 * Gửi dữ liệu sang E32.
 */
static RS485_Port_t uart2_e32;

static AppConfig_t *station_config;

static SensorRuntime_t sensor_runtime[
    APP_MAX_SENSORS];

static uint32_t previous_send_tick;

static uint8_t previous_config_mode =
    0xFFU;

static uint8_t StationApp_IsConfigMode(void)
{
    /*
     * PA1 = 0: Config.
     * PA1 = 1: Normal.
     */
    return
        (HAL_GPIO_ReadPin(
             GPIOA,
             GPIO_PIN_1) ==
         GPIO_PIN_RESET)
            ? 1U
            : 0U;
}

static uint8_t StationApp_ReadInputMask(void)
{
    uint8_t mask;

    mask = 0U;

    if (HAL_GPIO_ReadPin(
            IN1_GPIO_Port,
            IN1_Pin) ==
        INPUT_ACTIVE_STATE)
    {
        mask |= 0x01U;
    }

    if (HAL_GPIO_ReadPin(
            IN2_GPIO_Port,
            IN2_Pin) ==
        INPUT_ACTIVE_STATE)
    {
        mask |= 0x02U;
    }

    if (HAL_GPIO_ReadPin(
            IN3_GPIO_Port,
            IN3_Pin) ==
        INPUT_ACTIVE_STATE)
    {
        mask |= 0x04U;
    }

    if (HAL_GPIO_ReadPin(
            IN4_GPIO_Port,
            IN4_Pin) ==
        INPUT_ACTIVE_STATE)
    {
        mask |= 0x08U;
    }

    return mask;
}

static void StationApp_ClearRuntime(void)
{
    uint8_t index;

    for (index = 0U;
         index < APP_MAX_SENSORS;
         index++)
    {
        sensor_runtime[index].status =
            MODBUS_STATUS_TIMEOUT;

        memset(
            sensor_runtime[index].values,
            0,
            sizeof(
                sensor_runtime[index]
                    .values));
    }
}

static void StationApp_ReadAllSensors(void)
{
    uint8_t index;

    for (index = 0U;
         index < APP_MAX_SENSORS;
         index++)
    {
        AppSensorConfig_t *sensor =
            &station_config->
                sensors[index];

        memset(
            sensor_runtime[index].values,
            0,
            sizeof(
                sensor_runtime[index]
                    .values));

        if (sensor->enabled == 0U)
        {
            sensor_runtime[index].status =
                MODBUS_STATUS_BAD_PARAMETER;

            continue;
        }

        sensor_runtime[index].status =
            Modbus_ReadRegisters(
                &uart1_bus,
                sensor->slave_id,
                sensor->function_code,
                sensor->start_register,
                sensor->register_count,
                sensor_runtime[index].values,
                SENSOR_RETRY_COUNT);

        if (sensor_runtime[index].status !=
            MODBUS_STATUS_OK)
        {
            memset(
                sensor_runtime[index].values,
                0,
                sizeof(
                    sensor_runtime[index]
                        .values));
        }

        HAL_Delay(SENSOR_GAP_MS);
    }
}

static uint16_t StationApp_BuildPacket(
    uint8_t *packet,
    uint16_t packet_size)
{
    uint8_t sensor_index;
    uint8_t register_index;

    uint16_t offset;
    uint16_t total_length;
    uint16_t crc;

    if ((packet == NULL) ||
        (packet_size <
         PACKET_MAX_SIZE))
    {
        return 0U;
    }

    offset = 0U;

    packet[offset++] =
        PACKET_HEADER_0;

    packet[offset++] =
        PACKET_HEADER_1;

    /*
     * Length điền sau.
     */
    packet[offset++] = 0U;

    packet[offset++] =
        StationApp_ReadInputMask() &
        0x0FU;

    for (sensor_index = 0U;
         sensor_index <
             APP_MAX_SENSORS;
         sensor_index++)
    {
        AppSensorConfig_t *sensor =
            &station_config->
                sensors[sensor_index];

        if (sensor->enabled == 0U)
        {
            continue;
        }

        if ((offset +
             2U +
             (sensor->register_count *
              2U) +
             2U) >
            packet_size)
        {
            return 0U;
        }

        /*
         * Sensor block:
         *
         * Slave ID
         * Register count
         * Register data...
         */
        packet[offset++] =
            sensor->slave_id;

        packet[offset++] =
            sensor->register_count;

        for (register_index = 0U;
             register_index <
                 sensor->register_count;
             register_index++)
        {
            uint16_t value =
                sensor_runtime[
                    sensor_index]
                    .values[
                        register_index];

            packet[offset++] =
                (uint8_t)(
                    value >> 8U);

            packet[offset++] =
                (uint8_t)(
                    value & 0xFFU);
        }
    }

    total_length =
        offset + 2U;

    if (total_length > 255U)
    {
        return 0U;
    }

    packet[2] =
        total_length;

    crc =
        Modbus_CalculateCRC16(
            packet,
            offset);

    packet[offset++] =
        (uint8_t)(
            crc & 0xFFU);

    packet[offset++] =
        (uint8_t)(
            crc >> 8U);

    return offset;
}

static void StationApp_SendPacket(void)
{
    uint8_t packet[
        PACKET_MAX_SIZE];

    uint16_t packet_length;

    memset(
        packet,
        0,
        sizeof(packet));

    packet_length =
        StationApp_BuildPacket(
            packet,
            sizeof(packet));

    if (packet_length == 0U)
    {
        return;
    }

    (void)RS485_Send(
        &uart2_e32,
        packet,
        packet_length,
        1000U);
}

void StationApp_Init(
    UART_HandleTypeDef *uart1,
    UART_HandleTypeDef *uart2)
{
    if ((uart1 == NULL) ||
        (uart2 == NULL))
    {
        return;
    }

    /*
     * USART1 + MAX485:
     *
     * - USB-RS485 laptop
     * - cảm biến ID 01...08
     */
    RS485_PortInit(
        &uart1_bus,
        uart1,
        UART1_DE_GPIO_Port,
        UART1_DE_Pin,
        UART1_RE_GPIO_Port,
        UART1_RE_Pin);

    /*
     * USART2 + MAX485:
     *
     * E32.
     */
    RS485_PortInit(
        &uart2_e32,
        uart2,
        UART2_DE_GPIO_Port,
        UART2_DE_Pin,
        UART2_RE_GPIO_Port,
        UART2_RE_Pin);

    AppConfig_Init();

    station_config =
        AppConfig_Get();

    /*
     * AT và cảm biến cùng dùng uart1_bus.
     */
    AT_CommandInit(
        &uart1_bus,
        station_config,
        StationApp_ReadInputMask);

    StationApp_ClearRuntime();

    previous_send_tick =
        HAL_GetTick();

    previous_config_mode =
        0xFFU;

    RS485_SetReceiveMode(
        &uart1_bus);

    RS485_SetReceiveMode(
        &uart2_e32);
}

void StationApp_Task(void)
{
    uint8_t config_mode;
    uint32_t current_tick;

    config_mode =
        StationApp_IsConfigMode();

    /*
     * PA1 = 0:
     *
     * USART1 nhận AT từ USB-RS485.
     */
    if (config_mode != 0U)
    {
        if (previous_config_mode != 1U)
        {
            RS485_SetReceiveMode(
                &uart1_bus);

            AT_CommandNotifyReady();

            /*
             * PC13 active-low.
             */
            HAL_GPIO_WritePin(
                LED_GPIO_Port,
                LED_Pin,
                GPIO_PIN_RESET);
        }

        previous_config_mode = 1U;

        AT_CommandTask();

        return;
    }

    /*
     * PA1 = 1:
     *
     * USART1 đọc cảm biến.
     * USART2 gửi E32.
     */
    if (previous_config_mode != 0U)
    {
        previous_send_tick =
            HAL_GetTick();

        RS485_SetReceiveMode(
            &uart1_bus);

        RS485_SetReceiveMode(
            &uart2_e32);

        HAL_GPIO_WritePin(
            LED_GPIO_Port,
            LED_Pin,
            GPIO_PIN_SET);
    }

    previous_config_mode = 0U;

    if (station_config->role !=
        APP_ROLE_TX)
    {
        return;
    }

    current_tick =
        HAL_GetTick();

    if ((current_tick -
         previous_send_tick) <
        SEND_PERIOD_MS)
    {
        return;
    }

    previous_send_tick =
        current_tick;

    StationApp_ReadAllSensors();

    StationApp_SendPacket();

    HAL_GPIO_TogglePin(
        LED_GPIO_Port,
        LED_Pin);
}
