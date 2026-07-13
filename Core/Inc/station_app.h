#ifndef STATION_APP_H
#define STATION_APP_H

#include "main.h"

void StationApp_Init(
    UART_HandleTypeDef *sensor_uart,
    UART_HandleTypeDef *e32_uart
);

void StationApp_Task(void);

#endif
