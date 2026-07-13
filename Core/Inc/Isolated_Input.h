#ifndef INC_ISOLATED_INPUT_H_
#define INC_ISOLATED_INPUT_H_

#include "main.h"
#include <stdint.h>

/*
 * 0U = Active High
 * 1U = Active Low
 *
 * Phần cứng hiện tại:
 * 0 V   = không active
 * 3.3 V = active
 */
#define INPUT_ACTIVE_LOW    0U

typedef enum
{
    INPUT_1 = 0,
    INPUT_2,
    INPUT_3,
    INPUT_4

} InputChannel_t;

/*
 * Khởi tạo module Digital Input.
 * GPIO thực tế được cấu hình trong MX_GPIO_Init().
 */
void Input_Init(void);

/*
 * Đọc một input.
 *
 * Trả về:
 * 0U = không active
 * 1U = active
 */
uint8_t Input_GetState(
    InputChannel_t channel
);

/*
 * Đọc toàn bộ bốn input.
 *
 * state[0] = IN1
 * state[1] = IN2
 * state[2] = IN3
 * state[3] = IN4
 */
void Input_GetAll(
    uint8_t state[4]
);

/*
 * Đọc bốn input và đóng thành bit mask.
 *
 * bit0 = IN1
 * bit1 = IN2
 * bit2 = IN3
 * bit3 = IN4
 */
uint8_t Input_GetActiveMask(void);

#endif /* INC_ISOLATED_INPUT_H_ */
