#ifndef INC_STATION_SENSORS_H_
#define INC_STATION_SENSORS_H_

#include "modbus_master.h"
#include <stdint.h>

/*
 * Cau hinh theo thong tin hien tai:
 *
 * Gio RK100-01 RS485:
 * ID 1, function 03, register 0x0000, 1 register, x10 m/s.
 *
 * Mua RK400-04 RS485:
 * ID 2, function 03, register 0x0000, 1 register.
 * Bien rain_x10 luu gia tri raw theo protocol x10.
 *
 * Dat 7-in-1:
 * ID 3, function 03, register 0x0000, 7 register lien tiep.
 */

#define WIND_SLAVE_ID               1U
#define WIND_START_REGISTER         0x0000U
#define WIND_REGISTER_COUNT         1U

#define RAIN_SLAVE_ID               2U
#define RAIN_START_REGISTER         0x0000U
#define RAIN_REGISTER_COUNT         1U

#define SOIL_SLAVE_ID               3U
#define SOIL_START_REGISTER         0x0000U
#define SOIL_REGISTER_COUNT         7U

#define SENSOR_VALID_WIND           (1U << 0)
#define SENSOR_VALID_RAIN           (1U << 1)
#define SENSOR_VALID_SOIL           (1U << 2)

typedef struct
{
    /*
     * Gia tri so nguyen giu nguyen do phan giai,
     * tranh dung float tren STM32.
     */
    uint16_t wind_speed_x10;       /* 46 -> 4.6 m/s */
    uint16_t rainfall_x10;         /* 8 -> 0.8 mm */

    int16_t soil_temperature_x10;  /* -35 -> -3.5 C */
    uint16_t soil_moisture_x10;    /* 356 -> 35.6 % */
    uint16_t soil_ec_us_cm;        /* 1234 -> 1234 uS/cm */
    uint16_t soil_ph_x100;         /* 686 -> pH 6.86 */
    uint16_t soil_n_mg_kg;         /* 135 mg/kg */
    uint16_t soil_p_mg_kg;         /* 138 mg/kg */
    uint16_t soil_k_mg_kg;         /* 142 mg/kg */

    uint8_t valid_mask;

    ModbusStatus_t wind_status;
    ModbusStatus_t rain_status;
    ModbusStatus_t soil_status;
} StationSensorData_t;

void StationSensors_Init(RS485_Port_t *sensor_port);
void StationSensors_ReadAll(StationSensorData_t *data);

#endif
