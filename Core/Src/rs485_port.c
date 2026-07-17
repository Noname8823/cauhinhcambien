#include "rs485_port.h"

/* ============================================================
 * INTERNAL RX CLEAR
 * ============================================================ */

static void RS485_ClearRxAndErrors(
    RS485_Port_t *port)
{
    volatile uint32_t dummy;

    if ((port == NULL) ||
        (port->uart == NULL) ||
        (port->uart->Instance == NULL))
    {
        return;
    }

    /*
     * Đọc bỏ toàn bộ byte đang nằm trong DR.
     */
    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_RXNE) != RESET)
    {
        dummy =
            port->uart->Instance->DR;

        (void)dummy;
    }

    /*
     * STM32F1 xóa ORE/NE/FE/PE bằng trình tự:
     *
     * đọc SR rồi đọc DR.
     */
    dummy =
        port->uart->Instance->SR;

    dummy =
        port->uart->Instance->DR;

    (void)dummy;
}

/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void RS485_PortInit(
    RS485_Port_t *port,
    UART_HandleTypeDef *uart,
    GPIO_TypeDef *de_port,
    uint16_t de_pin,
    GPIO_TypeDef *re_port,
    uint16_t re_pin)
{
    if ((port == NULL) ||
        (uart == NULL) ||
        (de_port == NULL) ||
        (re_port == NULL))
    {
        return;
    }

    port->uart =
        uart;

    port->de_port =
        de_port;

    port->de_pin =
        de_pin;

    port->re_port =
        re_port;

    port->re_pin =
        re_pin;

    RS485_SetReceiveMode(
        port);

    RS485_ClearRxAndErrors(
        port);
}

/* ============================================================
 * MODE CONTROL
 * ============================================================ */

void RS485_SetTransmitMode(
    RS485_Port_t *port)
{
    if ((port == NULL) ||
        (port->de_port == NULL) ||
        (port->re_port == NULL))
    {
        return;
    }

    /*
     * Tắt receiver trước.
     *
     * /RE = 1: receiver disable.
     */
    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_SET);

    /*
     * Sau đó bật transmitter.
     *
     * DE = 1: transmitter enable.
     */
    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_SET);
}

void RS485_SetReceiveMode(
    RS485_Port_t *port)
{
    if ((port == NULL) ||
        (port->de_port == NULL) ||
        (port->re_port == NULL))
    {
        return;
    }

    /*
     * Tắt transmitter trước.
     *
     * DE = 0.
     */
    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_RESET);

    /*
     * Sau đó bật receiver.
     *
     * /RE = 0.
     */
    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_RESET);
}

/* ============================================================
 * RX FLUSH
 * ============================================================ */

void RS485_FlushRx(
    RS485_Port_t *port)
{
    RS485_ClearRxAndErrors(
        port);
}

/* ============================================================
 * SEND
 * ============================================================ */

RS485_Status_t RS485_Send(
    RS485_Port_t *port,
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint32_t start_tick;

    if ((port == NULL) ||
        (port->uart == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return RS485_STATUS_BAD_PARAMETER;
    }

    /*
     * Xóa dữ liệu cũ trước khi bật truyền.
     */
    RS485_ClearRxAndErrors(port);

    /*
     * /RE = 1, DE = 1.
     */
    RS485_SetTransmitMode(port);

    /*
     * Chờ MAX485 chuyển sang phát.
     */
    HAL_Delay(1U);

    hal_status = HAL_UART_Transmit(
        port->uart,
        (uint8_t *)data,
        length,
        timeout_ms);

    if (hal_status != HAL_OK)
    {
        RS485_SetReceiveMode(port);
        return RS485_STATUS_TX_ERROR;
    }

    /*
     * Chờ stop bit cuối cùng phát xong.
     */
    start_tick = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_TC) == RESET)
    {
        if ((timeout_ms != HAL_MAX_DELAY) &&
            ((HAL_GetTick() - start_tick) >= timeout_ms))
        {
            RS485_SetReceiveMode(port);
            return RS485_STATUS_TX_ERROR;
        }
    }

    /*
     * Quan trọng:
     * Nhả DE ngay và mở receiver ngay.
     * Không Delay và không Flush RX ở đoạn này.
     */
    RS485_SetReceiveMode(port);

    return RS485_STATUS_OK;
}
/* ============================================================
 * RECEIVE
 * ============================================================ */

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

    RS485_SetReceiveMode(
        port);

    hal_status =
        HAL_UART_Receive(
            port->uart,
            data,
            length,
            timeout_ms);

    if (hal_status ==
        HAL_TIMEOUT)
    {
        return RS485_STATUS_TIMEOUT;
    }

    if (hal_status !=
        HAL_OK)
    {
        RS485_ClearRxAndErrors(
            port);

        return RS485_STATUS_RX_ERROR;
    }

    return RS485_STATUS_OK;
}
