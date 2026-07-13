#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include "main.h"
#include "rs485_port.h"

typedef enum
{
    MODBUS_STATUS_OK = 0,

    MODBUS_STATUS_BAD_PARAMETER = 1,

    MODBUS_STATUS_TX_ERROR = 2,

    MODBUS_STATUS_TIMEOUT = 3,

    MODBUS_STATUS_BAD_RESPONSE = 4,

    MODBUS_STATUS_CRC_ERROR = 5,

    MODBUS_STATUS_EXCEPTION = 6

} ModbusStatus_t;


/*
 * Tính CRC16 Modbus.
 */
uint16_t Modbus_CalculateCRC16(
    const uint8_t *data,
    uint16_t length
);


/*
 * Đọc thanh ghi bằng function:
 *
 * 0x03: Read Holding Registers
 * 0x04: Read Input Registers
 */
ModbusStatus_t Modbus_ReadRegisters(
    RS485_Port_t *port,
    uint8_t slave_id,
    uint8_t function_code,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *register_buffer,
    uint8_t retry_count
);

#endif
