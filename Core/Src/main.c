#include "main.h"
#include "station_app.h"

/* =========================================================
 * BIẾN NGOẠI VI
 * ========================================================= */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* =========================================================
 * KHAI BÁO HÀM
 * ========================================================= */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* =========================================================
 * MAIN
 * ========================================================= */

int main(void)
{
    /*
     * Khởi tạo HAL.
     */
    HAL_Init();

    /*
     * Cấu hình clock hệ thống.
     */
    SystemClock_Config();

    /*
     * Cấu hình GPIO.
     */
    MX_GPIO_Init();

    /*
     * USART1:
     *
     * STM32
     * → MAX485 số 1
     * → bus cảm biến Modbus.
     */
    MX_USART1_UART_Init();

    /*
     * USART2:
     *
     * STM32
     * → MAX485 số 2
     * → E32 hoặc USB-RS485.
     */
    MX_USART2_UART_Init();

    /*
     * Khởi tạo ứng dụng.
     */
    StationApp_Init(
        &huart1,
        &huart2
    );

    while (1)
    {
        /*
         * Chạy task đọc cảm biến,
         * đọc Digital Input và gửi packet.
         */
        StationApp_Task();

        HAL_Delay(1U);
    }
}

/* =========================================================
 * SYSTEM CLOCK
 * ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /*
     * Dùng HSI nội 8 MHz.
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;

    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;

    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * SYSCLK = HSI = 8 MHz.
     */
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
            FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * USART1
 * ========================================================= */

static void MX_USART1_UART_Init(void)
{
    /*
     * Bus cảm biến Modbus:
     *
     * Baud rate : 9600
     * Data bit  : 8
     * Parity    : None
     * Stop bit  : 1
     */
    huart1.Instance =
        USART1;

    huart1.Init.BaudRate =
        9600;

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

/* =========================================================
 * USART2
 * ========================================================= */

static void MX_USART2_UART_Init(void)
{
    /*
     * Cổng E32 hoặc USB-RS485:
     *
     * Baud rate : 9600
     * Data bit  : 8
     * Parity    : None
     * Stop bit  : 1
     */
    huart2.Instance =
        USART2;

    huart2.Init.BaudRate =
        9600;

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

/* =========================================================
 * GPIO
 * ========================================================= */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* -----------------------------------------------------
     * Bật clock các GPIO
     * ----------------------------------------------------- */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* -----------------------------------------------------
     * Đặt mức output ban đầu
     * ----------------------------------------------------- */

    /*
     * MAX485 cảm biến:
     *
     * DE  = 0 → không phát.
     * /RE = 0 → cho phép nhận.
     */
    HAL_GPIO_WritePin(
        RS485_SENSOR_DE_GPIO_Port,
        RS485_SENSOR_DE_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        RS485_SENSOR_RE_GPIO_Port,
        RS485_SENSOR_RE_Pin,
        GPIO_PIN_RESET
    );

    /*
     * MAX485 E32:
     *
     * DE  = 0 → không phát.
     * /RE = 0 → cho phép nhận.
     */
    HAL_GPIO_WritePin(
        RS485_E32_DE_GPIO_Port,
        RS485_E32_DE_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        RS485_E32_RE_GPIO_Port,
        RS485_E32_RE_Pin,
        GPIO_PIN_RESET
    );

    /*
     * LED PC13 Blue Pill active-low:
     *
     * SET = LED tắt.
     */
    HAL_GPIO_WritePin(
        LED_GPIO_Port,
        LED_Pin,
        GPIO_PIN_SET
    );

    /*
     * RF mode mặc định bằng 0.
     */
    HAL_GPIO_WritePin(
        RF_MODE_GPIO_Port,
        RF_MODE_Pin,
        GPIO_PIN_RESET
    );

    /* -----------------------------------------------------
     * LED PC13
     * ----------------------------------------------------- */

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
        &GPIO_InitStruct
    );

    /* -----------------------------------------------------
     * MAX485 số 1: bus cảm biến
     *
     * DE  = PB0
     * /RE = PB1
     * ----------------------------------------------------- */

    GPIO_InitStruct.Pin =
        RS485_SENSOR_DE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        RS485_SENSOR_DE_GPIO_Port,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        RS485_SENSOR_RE_Pin;

    HAL_GPIO_Init(
        RS485_SENSOR_RE_GPIO_Port,
        &GPIO_InitStruct
    );

    /* -----------------------------------------------------
     * MAX485 số 2: E32 hoặc USB-RS485
     *
     * DE  = PA4
     * /RE = PA5
     * ----------------------------------------------------- */

    GPIO_InitStruct.Pin =
        RS485_E32_DE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        RS485_E32_DE_GPIO_Port,
        &GPIO_InitStruct
    );

    GPIO_InitStruct.Pin =
        RS485_E32_RE_Pin;

    HAL_GPIO_Init(
        RS485_E32_RE_GPIO_Port,
        &GPIO_InitStruct
    );

    /* -----------------------------------------------------
     * RF_MODE
     * ----------------------------------------------------- */

    GPIO_InitStruct.Pin =
        RF_MODE_Pin;

    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull =
        GPIO_NOPULL;

    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        RF_MODE_GPIO_Port,
        &GPIO_InitStruct
    );

    /* -----------------------------------------------------
     * 4 Digital Input active-high
     *
     * PB5 = IN1
     * PB6 = IN2
     * PB7 = IN3
     * PB8 = IN4
     *
     * Không kích:
     * GPIO_PULLDOWN giữ chân ở mức 0.
     *
     * Kích:
     * đưa 3.3 V vào chân → mức 1.
     * ----------------------------------------------------- */

    GPIO_InitStruct.Mode =
        GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull =
        GPIO_PULLDOWN;

    /*
     * IN1.
     */
    GPIO_InitStruct.Pin =
        IN1_Pin;

    HAL_GPIO_Init(
        IN1_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * IN2.
     */
    GPIO_InitStruct.Pin =
        IN2_Pin;

    HAL_GPIO_Init(
        IN2_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * IN3.
     */
    GPIO_InitStruct.Pin =
        IN3_Pin;

    HAL_GPIO_Init(
        IN3_GPIO_Port,
        &GPIO_InitStruct
    );

    /*
     * IN4.
     */
    GPIO_InitStruct.Pin =
        IN4_Pin;

    HAL_GPIO_Init(
        IN4_GPIO_Port,
        &GPIO_InitStruct
    );
}

/* =========================================================
 * ERROR HANDLER
 * ========================================================= */

void Error_Handler(void)
{
    /*
     * Không gọi __disable_irq().
     *
     * HAL_Delay() cần SysTick interrupt hoạt động.
     */
    while (1)
    {
        HAL_GPIO_TogglePin(
            LED_GPIO_Port,
            LED_Pin
        );

        HAL_Delay(100U);
    }
}

/* =========================================================
 * ASSERT
 * ========================================================= */

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
