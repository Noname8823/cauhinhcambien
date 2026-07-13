#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /*
     * Giữ SWD để debug,
     * tắt JTAG đầy đủ nhằm giải phóng chân.
     */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_UART_MspInit(
    UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*
         * PA9 = USART1_TX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_9;

        GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;

        GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_HIGH;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );

        /*
         * PA10 = USART1_RX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_10;

        GPIO_InitStruct.Mode =
            GPIO_MODE_INPUT;

        GPIO_InitStruct.Pull =
            GPIO_NOPULL;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );
    }
    else if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /*
         * PA2 = USART2_TX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_2;

        GPIO_InitStruct.Mode =
            GPIO_MODE_AF_PP;

        GPIO_InitStruct.Speed =
            GPIO_SPEED_FREQ_HIGH;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );

        /*
         * PA3 = USART2_RX
         */
        GPIO_InitStruct.Pin =
            GPIO_PIN_3;

        GPIO_InitStruct.Mode =
            GPIO_MODE_INPUT;

        GPIO_InitStruct.Pull =
            GPIO_NOPULL;

        HAL_GPIO_Init(
            GPIOA,
            &GPIO_InitStruct
        );
    }
}

void HAL_UART_MspDeInit(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();

        HAL_GPIO_DeInit(
            GPIOA,
            GPIO_PIN_9 | GPIO_PIN_10
        );
    }
    else if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();

        HAL_GPIO_DeInit(
            GPIOA,
            GPIO_PIN_2 | GPIO_PIN_3
        );
    }
}
