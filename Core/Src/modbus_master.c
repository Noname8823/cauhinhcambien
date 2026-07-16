#include "modbus_master.h"

#include <string.h>

#define MODBUS_REQUEST_SIZE       8U
#define MODBUS_RESPONSE_TIMEOUT   600U
#define MODBUS_RETRY_DELAY_MS     20U

uint16_t Modbus_CalculateCRC16(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    crc = 0xFFFFU;

    for (index = 0U;
         index < length;
         index++)
    {
        crc ^= data[index];

        for (bit = 0U;
             bit < 8U;
             bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc =
                    (uint16_t)(
                        (crc >> 1U) ^
                        0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static ModbusStatus_t
Modbus_ReadRegistersOnce(
    RS485_Port_t *port,
    uint8_t slave_id,
    uint8_t function_code,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *register_values)
{
    uint8_t request[
        MODBUS_REQUEST_SIZE];

    uint8_t response[
        3U +
        (MODBUS_MAX_REGISTERS * 2U) +
        2U];

    uint16_t request_crc;
    uint16_t received_crc;
    uint16_t calculated_crc;

    uint16_t expected_data_length;
    uint16_t response_length;
    uint16_t index;

    RS485_Status_t rs485_status;

    if ((port == NULL) ||
        (register_values == NULL) ||
        (slave_id == 0U) ||
        (register_count == 0U) ||
        (register_count >
         MODBUS_MAX_REGISTERS))
    {
        return MODBUS_STATUS_BAD_PARAMETER;
    }

    if ((function_code != 0x03U) &&
        (function_code != 0x04U))
    {
        return MODBUS_STATUS_BAD_PARAMETER;
    }

    memset(
        register_values,
        0,
        register_count *
            sizeof(uint16_t));

    /*
     * Modbus RTU request:
     *
     * Slave
     * Function
     * Start high
     * Start low
     * Count high
     * Count low
     * CRC low
     * CRC high
     */
    request[0] = slave_id;
    request[1] = function_code;

    request[2] =
        (uint8_t)(
            start_register >> 8U);

    request[3] =
        (uint8_t)(
            start_register & 0xFFU);

    request[4] =
        (uint8_t)(
            register_count >> 8U);

    request[5] =
        (uint8_t)(
            register_count & 0xFFU);

    request_crc =
        Modbus_CalculateCRC16(
            request,
            6U);

    request[6] =
        (uint8_t)(
            request_crc & 0xFFU);

    request[7] =
        (uint8_t)(
            request_crc >> 8U);

    RS485_FlushRx(port);

    rs485_status =
        RS485_Send(
            port,
            request,
            sizeof(request),
            MODBUS_RESPONSE_TIMEOUT);

    if (rs485_status !=
        RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TX_ERROR;
    }

    /*
     * Đọc trước:
     *
     * Slave
     * Function
     * Byte count hoặc exception code
     */
    rs485_status =
        RS485_Receive(
            port,
            response,
            3U,
            MODBUS_RESPONSE_TIMEOUT);

    if (rs485_status !=
        RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TIMEOUT;
    }

    if (response[0] != slave_id)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }

    /*
     * Modbus exception.
     */
    if (response[1] ==
        (uint8_t)(
            function_code | 0x80U))
    {
        rs485_status =
            RS485_Receive(
                port,
                &response[3],
                2U,
                MODBUS_RESPONSE_TIMEOUT);

        if (rs485_status !=
            RS485_STATUS_OK)
        {
            return MODBUS_STATUS_TIMEOUT;
        }

        received_crc =
            (uint16_t)response[3] |
            ((uint16_t)response[4]
             << 8U);

        calculated_crc =
            Modbus_CalculateCRC16(
                response,
                3U);

        if (received_crc !=
            calculated_crc)
        {
            return MODBUS_STATUS_CRC_ERROR;
        }

        return MODBUS_STATUS_EXCEPTION;
    }

    if (response[1] !=
        function_code)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }

    expected_data_length =
        register_count * 2U;

    if (response[2] !=
        expected_data_length)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }

    response_length =
        expected_data_length + 2U;

    rs485_status =
        RS485_Receive(
            port,
            &response[3],
            response_length,
            MODBUS_RESPONSE_TIMEOUT);

    if (rs485_status !=
        RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TIMEOUT;
    }

    received_crc =
        (uint16_t)
            response[
                3U +
                expected_data_length] |
        ((uint16_t)
            response[
                4U +
                expected_data_length]
         << 8U);

    calculated_crc =
        Modbus_CalculateCRC16(
            response,
            (uint16_t)(
                3U +
                expected_data_length));

    if (received_crc !=
        calculated_crc)
    {
        return MODBUS_STATUS_CRC_ERROR;
    }

    for (index = 0U;
         index < register_count;
         index++)
    {
        register_values[index] =
            ((uint16_t)
                response[
                    3U +
                    (index * 2U)]
             << 8U) |
            response[
                4U +
                (index * 2U)];
    }

    return MODBUS_STATUS_OK;
}

ModbusStatus_t Modbus_ReadRegisters(
    RS485_Port_t *port,
    uint8_t slave_id,
    uint8_t function_code,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *register_values,
    uint8_t retry_count)
{
    ModbusStatus_t status;
    uint8_t attempt;

    status =
        MODBUS_STATUS_TIMEOUT;

    for (attempt = 0U;
         attempt <= retry_count;
         attempt++)
    {
        status =
            Modbus_ReadRegistersOnce(
                port,
                slave_id,
                function_code,
                start_register,
                register_count,
                register_values);

        if (status ==
            MODBUS_STATUS_OK)
        {
            return status;
        }

        memset(
            register_values,
            0,
            register_count *
                sizeof(uint16_t));

        if (attempt < retry_count)
        {
            HAL_Delay(
                MODBUS_RETRY_DELAY_MS);
        }
    }

    return status;
}
