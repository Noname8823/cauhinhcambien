#include "rs485_port.h"

void RS485_PortInit(
    RS485_Port_t *port,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *de_port,
    uint16_t de_pin,
    GPIO_TypeDef *re_port,
    uint16_t re_pin)
{
    if (port == NULL)
    {
        return;
    }

    port->uart = uart;

    port->de_port = de_port;
    port->de_pin = de_pin;

    port->re_port = re_port;
    port->re_pin = re_pin;

    RS485_SetReceiveMode(port);
}

void RS485_SetTransmitMode(
    RS485_Port_t *port)
{
    if (port == NULL)
    {
        return;
    }

    /*
     * MAX485:
     *
     * DE  = 1: bật driver truyền.
     * /RE = 1: tắt receiver.
     */
    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_SET);
}

void RS485_SetReceiveMode(
    RS485_Port_t *port)
{
    if (port == NULL)
    {
        return;
    }

    /*
     * MAX485:
     *
     * DE  = 0: tắt driver truyền.
     * /RE = 0: bật receiver.
     */
    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_RESET);
}

void RS485_FlushRx(
    RS485_Port_t *port)
{
    volatile uint32_t dummy;

    if ((port == NULL) ||
        (port->uart == NULL))
    {
        return;
    }

    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_RXNE) != RESET)
    {
        dummy =
            port->uart->Instance->DR;

        (void)dummy;
    }

    __HAL_UART_CLEAR_OREFLAG(
        port->uart);
}

RS485_Status_t RS485_Send(
    RS485_Port_t *port,
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((port == NULL) ||
        (port->uart == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return RS485_STATUS_BAD_PARAMETER;
    }

    RS485_SetTransmitMode(port);

    /*
     * Cho MAX485 đủ thời gian chuyển chế độ.
     */
    HAL_Delay(1U);

    hal_status =
        HAL_UART_Transmit(
            port->uart,
            (uint8_t *)data,
            length,
            timeout_ms);

    /*
     * Chờ byte cuối thật sự ra khỏi UART.
     */
    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_TC) == RESET)
    {
    }

    HAL_Delay(1U);

    RS485_SetReceiveMode(port);

    if (hal_status != HAL_OK)
    {
        return RS485_STATUS_TX_ERROR;
    }

    return RS485_STATUS_OK;
}

RS485_Status_t RS485_Receive(
    RS485_Port_t *port,
    uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if ((port == NULL) ||
        (port->uart == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return RS485_STATUS_BAD_PARAMETER;
    }

    RS485_SetReceiveMode(port);

    hal_status =
        HAL_UART_Receive(
            port->uart,
            data,
            length,
            timeout_ms);

    if (hal_status == HAL_TIMEOUT)
    {
        return RS485_STATUS_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return RS485_STATUS_RX_ERROR;
    }

    return RS485_STATUS_OK;
}
