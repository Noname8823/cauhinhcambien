#include "main.h"
#include "station_app.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();

    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    StationApp_Init(
        &huart1,
        &huart2);

    while (1)
    {
        StationApp_Task();

        HAL_Delay(1U);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef
        RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef
        RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) !=
        HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0) !=
        HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;

    huart1.Init.BaudRate = 9600;

    huart1.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart1.Init.StopBits =
        UART_STOPBITS_1;

    huart1.Init.Parity =
        UART_PARITY_NONE;

    huart1.Init.Mode =
        UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart1.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(
            &huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 9600;

    huart2.Init.WordLength =
        UART_WORDLENGTH_8B;

    huart2.Init.StopBits =
        UART_STOPBITS_1;

    huart2.Init.Parity =
        UART_PARITY_NONE;

    huart2.Init.Mode =
        UART_MODE_TX_RX;

    huart2.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;

    huart2.Init.OverSampling =
        UART_OVERSAMPLING_16;

    if (HAL_UART_Init(
            &huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /*
     * Trạng thái mặc định MAX485 USART1:
     *
     * DE  = 0: không phát.
     * /RE = 0: cho phép nhận.
     */
    HAL_GPIO_WritePin(
        GPIOB,
        UART1_DE_Pin | UART1_RE_Pin,
        GPIO_PIN_RESET);

    /*
     * Trạng thái mặc định MAX485 USART2.
     */
    HAL_GPIO_WritePin(
        GPIOA,
        UART2_DE_Pin | UART2_RE_Pin,
        GPIO_PIN_RESET);

    /*
     * LED PC13.
     */
    HAL_GPIO_WritePin(
        LED_GPIO_Port,
        LED_Pin,
        GPIO_PIN_SET);

    /*
     * UART1 DE và /RE:
     * PB1 và PB10.
     */
    GPIO_InitStruct.Pin =
        UART1_DE_Pin |
        UART1_RE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct);

    /*
     * UART2 DE và /RE:
     * PA4 và PA5.
     */
    GPIO_InitStruct.Pin =
        UART2_DE_Pin |
        UART2_RE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct);

    /*
     * PA1: Config Mode.
     * PA1=0: cấu hình AT.
     * PA1=1: normal.
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct);

    /*
     * LED PC13.
     */
    GPIO_InitStruct.Pin =
        LED_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        LED_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * Digital input PB5..PB8.
     */
    GPIO_InitStruct.Pin =
        IN1_Pin |
        IN2_Pin |
        IN3_Pin |
        IN4_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLDOWN;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct);


    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*
     * Trạng thái output mặc định.
     */
    HAL_GPIO_WritePin(
        UART1_DE_GPIO_Port,
        UART1_DE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        UART1_RE_GPIO_Port,
        UART1_RE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        UART2_DE_GPIO_Port,
        UART2_DE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        UART2_RE_GPIO_Port,
        UART2_RE_Pin,
        GPIO_PIN_RESET);

    HAL_GPIO_WritePin(
        LED_GPIO_Port,
        LED_Pin,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        RF_MODE_GPIO_Port,
        RF_MODE_Pin,
        GPIO_PIN_RESET);

    /*
     * LED.
     */
    GPIO_InitStruct.Pin =
        LED_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        LED_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * UART1 DE.
     */
    GPIO_InitStruct.Pin =
        UART1_DE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        UART1_DE_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * UART1 /RE.
     */
    GPIO_InitStruct.Pin =
        UART1_RE_Pin;

    HAL_GPIO_Init(
        UART1_RE_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * UART2 DE.
     */
    GPIO_InitStruct.Pin =
        UART2_DE_Pin;

    HAL_GPIO_Init(
        UART2_DE_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * UART2 /RE.
     */
    GPIO_InitStruct.Pin =
        UART2_RE_Pin;

    HAL_GPIO_Init(
        UART2_RE_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * RF mode.
     */
    GPIO_InitStruct.Pin =
        RF_MODE_Pin;

    HAL_GPIO_Init(
        RF_MODE_GPIO_Port,
        &GPIO_InitStruct);

    /*
     * PA1:
     *
     * 0 = config.
     * 1 = normal.
     *
     * Để hở sẽ được kéo lên mức 1.
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_1;

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLUP;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct);

    /*
     * Digital inputs:
     * mức 1 là active.
     */
    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLDOWN;

    GPIO_InitStruct.Pin =
        IN1_Pin;

    HAL_GPIO_Init(
        IN1_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        IN2_Pin;

    HAL_GPIO_Init(
        IN2_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        IN3_Pin;

    HAL_GPIO_Init(
        IN3_GPIO_Port,
        &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        IN4_Pin;

    HAL_GPIO_Init(
        IN4_GPIO_Port,
        &GPIO_InitStruct);
}

void Error_Handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(
            LED_GPIO_Port,
            LED_Pin);

        HAL_Delay(100U);
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
