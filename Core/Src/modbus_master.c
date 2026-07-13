#include "modbus_master.h"

#include <string.h>


#define MODBUS_MAX_REGISTERS           32U

#define MODBUS_REQUEST_SIZE            8U

#define MODBUS_RESPONSE_TIMEOUT_MS     500U

#define MODBUS_RETRY_DELAY_MS          20U


uint16_t Modbus_CalculateCRC16(
    const uint8_t *data,
    uint16_t length)
{
    uint16_t crc;
    uint16_t i;
    uint8_t bit;

    if (data == NULL)
    {
        return 0U;
    }

    crc = 0xFFFFU;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc =
                    (uint16_t)(
                        (crc >> 1U) ^
                        0xA001U
                    );
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}


static ModbusStatus_t Modbus_ReadOnce(
    RS485_Port_t *port,
    uint8_t slave_id,
    uint8_t function_code,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *register_buffer)
{
    uint8_t request[MODBUS_REQUEST_SIZE];

    /*
     * Response tối đa:
     *
     * ID                 1
     * Function           1
     * Byte count         1
     * Data               64
     * CRC                2
     *
     * Tổng tối đa 69 byte.
     */
    uint8_t response[
        3U +
        (MODBUS_MAX_REGISTERS * 2U) +
        2U
    ];

    uint16_t request_crc;
    uint16_t received_crc;
    uint16_t calculated_crc;

    uint16_t response_length;
    uint16_t expected_byte_count;

    uint16_t i;

    uint8_t byte_count;

    RS485_Status_t rs485_status;


    /*
     * Tạo request Modbus RTU.
     */

    request[0] = slave_id;

    request[1] = function_code;

    request[2] =
        (uint8_t)(
            start_register >> 8U
        );

    request[3] =
        (uint8_t)(
            start_register & 0xFFU
        );

    request[4] =
        (uint8_t)(
            register_count >> 8U
        );

    request[5] =
        (uint8_t)(
            register_count & 0xFFU
        );

    request_crc =
        Modbus_CalculateCRC16(
            request,
            6U
        );

    /*
     * Modbus CRC gửi byte thấp trước.
     */
    request[6] =
        (uint8_t)(
            request_crc & 0xFFU
        );

    request[7] =
        (uint8_t)(
            request_crc >> 8U
        );


    /*
     * Xóa byte cũ trước khi gửi request mới.
     */
    RS485_FlushRx(port);


    rs485_status = RS485_Send(
        port,
        request,
        sizeof(request),
        MODBUS_RESPONSE_TIMEOUT_MS
    );

    if (rs485_status != RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TX_ERROR;
    }


    /*
     * Đọc trước 3 byte:
     *
     * byte 0 = Slave ID
     * byte 1 = Function
     * byte 2 = Byte count hoặc exception code
     */
    rs485_status = RS485_Receive(
        port,
        response,
        3U,
        MODBUS_RESPONSE_TIMEOUT_MS
    );

    if (rs485_status != RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TIMEOUT;
    }


    /*
     * Kiểm tra Slave ID.
     */
    if (response[0] != slave_id)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }


    /*
     * Cảm biến trả exception:
     *
     * function trả về = function gửi + 0x80.
     *
     * Ví dụ:
     * gửi 0x03
     * lỗi trả 0x83.
     */
    if (response[1] ==
        (uint8_t)(function_code | 0x80U))
    {
        /*
         * Đã đọc:
         * ID, function, exception code.
         *
         * Còn đọc 2 byte CRC.
         */
        rs485_status = RS485_Receive(
            port,
            &response[3],
            2U,
            MODBUS_RESPONSE_TIMEOUT_MS
        );

        if (rs485_status != RS485_STATUS_OK)
        {
            return MODBUS_STATUS_TIMEOUT;
        }

        received_crc =
            (uint16_t)response[3] |
            ((uint16_t)response[4] << 8U);

        calculated_crc =
            Modbus_CalculateCRC16(
                response,
                3U
            );

        if (received_crc != calculated_crc)
        {
            return MODBUS_STATUS_CRC_ERROR;
        }

        return MODBUS_STATUS_EXCEPTION;
    }


    /*
     * Function trả về phải giống function gửi.
     */
    if (response[1] != function_code)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }


    byte_count = response[2];

    expected_byte_count =
        register_count * 2U;

    /*
     * Mỗi register là 2 byte.
     */
    if ((uint16_t)byte_count !=
        expected_byte_count)
    {
        return MODBUS_STATUS_BAD_RESPONSE;
    }


    /*
     * Đọc:
     *
     * byte_count byte dữ liệu
     * + 2 byte CRC.
     */
    rs485_status = RS485_Receive(
        port,
        &response[3],
        (uint16_t)byte_count + 2U,
        MODBUS_RESPONSE_TIMEOUT_MS
    );

    if (rs485_status != RS485_STATUS_OK)
    {
        return MODBUS_STATUS_TIMEOUT;
    }


    response_length =
        3U +
        (uint16_t)byte_count +
        2U;


    received_crc =
        (uint16_t)response[
            response_length - 2U
        ] |
        (
            (uint16_t)response[
                response_length - 1U
            ] << 8U
        );


    calculated_crc =
        Modbus_CalculateCRC16(
            response,
            response_length - 2U
        );


    if (received_crc != calculated_crc)
    {
        return MODBUS_STATUS_CRC_ERROR;
    }


    /*
     * Ghép từng cặp byte thành register 16-bit.
     *
     * Modbus gửi byte cao trước.
     */
    for (i = 0U;
         i < register_count;
         i++)
    {
        register_buffer[i] =
            (
                (uint16_t)response[
                    3U + (i * 2U)
                ] << 8U
            ) |
            (
                (uint16_t)response[
                    4U + (i * 2U)
                ]
            );
    }


    return MODBUS_STATUS_OK;
}


ModbusStatus_t Modbus_ReadRegisters(
    RS485_Port_t *port,
    uint8_t slave_id,
    uint8_t function_code,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *register_buffer,
    uint8_t retry_count)
{
    ModbusStatus_t status;
    uint8_t attempt;

    if ((port == NULL) ||
        (register_buffer == NULL) ||
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

    status = MODBUS_STATUS_TIMEOUT;

    /*
     * retry_count = 2:
     *
     * lần đầu + thử lại 2 lần
     * tổng cộng tối đa 3 lần.
     */
    for (attempt = 0U;
         attempt <= retry_count;
         attempt++)
    {
        status = Modbus_ReadOnce(
            port,
            slave_id,
            function_code,
            start_register,
            register_count,
            register_buffer
        );

        if (status == MODBUS_STATUS_OK)
        {
            return MODBUS_STATUS_OK;
        }

        if (attempt < retry_count)
        {
            HAL_Delay(
                MODBUS_RETRY_DELAY_MS
            );
        }
    }

    return status;
}
