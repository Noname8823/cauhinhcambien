#ifndef RS485_PORT_H
#define RS485_PORT_H

#include "main.h"

typedef enum
{
    RS485_STATUS_OK = 0,
    RS485_STATUS_BAD_PARAMETER,
    RS485_STATUS_TX_ERROR,
    RS485_STATUS_RX_ERROR,
    RS485_STATUS_TIMEOUT
} RS485_Status_t;

typedef struct
{
    UART_HandleTypeDef *uart;

    GPIO_TypeDef *de_port;
    uint16_t de_pin;

    GPIO_TypeDef *re_port;
    uint16_t re_pin;
} RS485_Port_t;

void RS485_PortInit(
    RS485_Port_t *port,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *de_port,
    uint16_t de_pin,
    GPIO_TypeDef *re_port,
    uint16_t re_pin
);

void RS485_SetTransmitMode(
    RS485_Port_t *port
);

void RS485_SetReceiveMode(
    RS485_Port_t *port
);

void RS485_FlushRx(
    RS485_Port_t *port
);

RS485_Status_t RS485_Send(
    RS485_Port_t *port,
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms
);

RS485_Status_t RS485_Receive(
    RS485_Port_t *port,
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms
);

#endif
