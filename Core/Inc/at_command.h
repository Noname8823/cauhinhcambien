#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#include "main.h"
#include "rs485_port.h"
#include "app_config.h"

typedef uint8_t (*AT_InputMaskReader_t)(void);

void AT_CommandInit(
    RS485_Port_t *shared_uart1_bus,
    AppConfig_t *config,
    AT_InputMaskReader_t input_reader
);

void AT_CommandTask(void);

void AT_CommandNotifyReady(void);

/*
 * Xóa dòng AT nhận dở và xóa dữ liệu cũ trong UART RX.
 */
void AT_CommandResetReceiver(void);

#endif
