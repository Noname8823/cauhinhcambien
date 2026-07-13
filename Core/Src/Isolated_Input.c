#include "Isolated_Input.h"

void Input_Init(void)
{
    /*
     * Không cần cấu hình lại GPIO tại đây.
     *
     * GPIO đã được cấu hình trong:
     * MX_GPIO_Init()
     */
}

static GPIO_PinState Input_ReadRaw(
    InputChannel_t channel)
{
    switch (channel)
    {
        case INPUT_1:
            return HAL_GPIO_ReadPin(
                IN1_GPIO_Port,
                IN1_Pin
            );

        case INPUT_2:
            return HAL_GPIO_ReadPin(
                IN2_GPIO_Port,
                IN2_Pin
            );

        case INPUT_3:
            return HAL_GPIO_ReadPin(
                IN3_GPIO_Port,
                IN3_Pin
            );

        case INPUT_4:
            return HAL_GPIO_ReadPin(
                IN4_GPIO_Port,
                IN4_Pin
            );

        default:
            return GPIO_PIN_RESET;
    }
}

uint8_t Input_GetState(
    InputChannel_t channel)
{
    GPIO_PinState raw_state;

    raw_state =
        Input_ReadRaw(channel);

#if (INPUT_ACTIVE_LOW == 1U)

    /*
     * Active-low:
     *
     * GPIO = 0 → active.
     * GPIO = 1 → không active.
     */
    if (raw_state == GPIO_PIN_RESET)
    {
        return 1U;
    }

#else

    /*
     * Active-high:
     *
     * GPIO = 1 → active.
     * GPIO = 0 → không active.
     */
    if (raw_state == GPIO_PIN_SET)
    {
        return 1U;
    }

#endif

    return 0U;
}

void Input_GetAll(
    uint8_t state[4])
{
    if (state == NULL)
    {
        return;
    }

    state[0] =
        Input_GetState(INPUT_1);

    state[1] =
        Input_GetState(INPUT_2);

    state[2] =
        Input_GetState(INPUT_3);

    state[3] =
        Input_GetState(INPUT_4);
}

uint8_t Input_GetActiveMask(void)
{
    uint8_t mask = 0U;

    /*
     * bit0 = IN1
     */
    if (Input_GetState(INPUT_1) != 0U)
    {
        mask |= 0x01U;
    }

    /*
     * bit1 = IN2
     */
    if (Input_GetState(INPUT_2) != 0U)
    {
        mask |= 0x02U;
    }

    /*
     * bit2 = IN3
     */
    if (Input_GetState(INPUT_3) != 0U)
    {
        mask |= 0x04U;
    }

    /*
     * bit3 = IN4
     */
    if (Input_GetState(INPUT_4) != 0U)
    {
        mask |= 0x08U;
    }

    return mask;
}
