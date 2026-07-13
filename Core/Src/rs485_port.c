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

    /*
     * Mặc định đặt MAX485 ở chế độ nhận.
     */
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
     * DE  = 1: bật bộ phát
     * /RE = 1: tắt bộ nhận
     */

    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_SET
    );
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
     * DE  = 0: tắt bộ phát
     * /RE = 0: bật bộ nhận
     */

    HAL_GPIO_WritePin(
        port->de_port,
        port->de_pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        port->re_port,
        port->re_pin,
        GPIO_PIN_RESET
    );
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

    /*
     * Đọc bỏ các byte cũ còn trong UART.
     */
    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_RXNE) != RESET)
    {
        dummy = port->uart->Instance->DR;
        (void)dummy;
    }

    /*
     * Xóa lỗi overrun nếu có.
     */
    if (__HAL_UART_GET_FLAG(
            port->uart,
            UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(
            port->uart
        );
    }
}


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

    RS485_SetTransmitMode(port);

    /*
     * Chờ MAX485 chuyển sang chế độ phát.
     */
    HAL_Delay(1U);

    hal_status = HAL_UART_Transmit(
        port->uart,
        (uint8_t *)data,
        length,
        timeout_ms
    );

    if (hal_status != HAL_OK)
    {
        RS485_SetReceiveMode(port);

        if (hal_status == HAL_TIMEOUT)
        {
            return RS485_STATUS_TIMEOUT;
        }

        return RS485_STATUS_TX_ERROR;
    }

    /*
     * Chờ byte cuối cùng thực sự ra khỏi chân TX.
     */
    start_tick = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(
               port->uart,
               UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start_tick) >=
            timeout_ms)
        {
            RS485_SetReceiveMode(port);

            return RS485_STATUS_TIMEOUT;
        }
    }

    /*
     * Sau khi gửi xong, chuyển ngay về nhận.
     */
    RS485_SetReceiveMode(port);

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

    hal_status = HAL_UART_Receive(
        port->uart,
        data,
        length,
        timeout_ms
    );

    if (hal_status == HAL_OK)
    {
        return RS485_STATUS_OK;
    }

    if (hal_status == HAL_TIMEOUT)
    {
        return RS485_STATUS_TIMEOUT;
    }

    return RS485_STATUS_RX_ERROR;
}
