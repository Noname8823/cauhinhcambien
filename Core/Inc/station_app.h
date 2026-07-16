#ifndef STATION_APP_H
#define STATION_APP_H

#include "main.h"

void StationApp_Init(
    UART_HandleTypeDef *uart1,
    UART_HandleTypeDef *uart2
);

void StationApp_Task(void);

#endif
