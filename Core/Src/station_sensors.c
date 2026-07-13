#include "station_sensors.h"
#include <string.h>

#define SENSOR_READ_RETRY_COUNT  2U
#define SENSOR_GAP_MS            20U

static RS485_Port_t *g_sensor_port = NULL;

void StationSensors_Init(RS485_Port_t *sensor_port)
{
    g_sensor_port = sensor_port;
}

static void StationSensors_ReadWind(StationSensorData_t *data)
{
    uint16_t registers[WIND_REGISTER_COUNT];

    data->wind_status =
        Modbus_ReadHoldingRegisters(g_sensor_port,
                                    WIND_SLAVE_ID,
                                    WIND_START_REGISTER,
                                    WIND_REGISTER_COUNT,
                                    registers,
                                    SENSOR_READ_RETRY_COUNT);

    if (data->wind_status == MODBUS_STATUS_OK)
    {
        data->wind_speed_x10 = registers[0];
        data->valid_mask |= SENSOR_VALID_WIND;
    }
}

static void StationSensors_ReadRain(StationSensorData_t *data)
{
    uint16_t registers[RAIN_REGISTER_COUNT];

    data->rain_status =
        Modbus_ReadHoldingRegisters(g_sensor_port,
                                    RAIN_SLAVE_ID,
                                    RAIN_START_REGISTER,
                                    RAIN_REGISTER_COUNT,
                                    registers,
                                    SENSOR_READ_RETRY_COUNT);

    if (data->rain_status == MODBUS_STATUS_OK)
    {
        data->rainfall_x10 = registers[0];
        data->valid_mask |= SENSOR_VALID_RAIN;
    }
}

static void StationSensors_ReadSoil(StationSensorData_t *data)
{
    uint16_t registers[SOIL_REGISTER_COUNT];

    data->soil_status =
        Modbus_ReadHoldingRegisters(g_sensor_port,
                                    SOIL_SLAVE_ID,
                                    SOIL_START_REGISTER,
                                    SOIL_REGISTER_COUNT,
                                    registers,
                                    SENSOR_READ_RETRY_COUNT);

    if (data->soil_status == MODBUS_STATUS_OK)
    {
        /*
         * Thu tu thanh ghi dua tren bang ban da chup:
         *
         * R0: nhiet do dat, int16 two's complement, chia 10.
         * R1: do am dat, chia 10.
         * R2: EC, uS/cm.
         * R3: pH, chia 100.
         * R4: N, mg/kg.
         * R5: P, mg/kg.
         * R6: K, mg/kg.
         */
        data->soil_temperature_x10 = (int16_t)registers[0];
        data->soil_moisture_x10 = registers[1];
        data->soil_ec_us_cm = registers[2];
        data->soil_ph_x100 = registers[3];
        data->soil_n_mg_kg = registers[4];
        data->soil_p_mg_kg = registers[5];
        data->soil_k_mg_kg = registers[6];

        data->valid_mask |= SENSOR_VALID_SOIL;
    }
}

void StationSensors_ReadAll(StationSensorData_t *data)
{
    if ((data == NULL) || (g_sensor_port == NULL))
    {
        return;
    }

    memset(data, 0, sizeof(*data));

    /*
     * Doc theo thu tu ID 1 -> 2 -> 3.
     * Tai moi thoi diem chi mot slave duoc phep tra loi.
     */
    StationSensors_ReadWind(data);
    HAL_Delay(SENSOR_GAP_MS);

    StationSensors_ReadRain(data);
    HAL_Delay(SENSOR_GAP_MS);

    StationSensors_ReadSoil(data);
}
