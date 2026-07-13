#include "telemetry_packet.h"
#include "modbus_master.h"
#include <string.h>

static void Telemetry_WriteU16LE(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)(value >> 8U);
}

void Telemetry_BuildPacket(uint8_t packet[TELEMETRY_PACKET_SIZE],
                           uint8_t sequence,
                           uint8_t node_id,
                           uint8_t digital_input_mask,
                           const StationSensorData_t *sensor_data)
{
    uint16_t crc;

    if ((packet == NULL) || (sensor_data == NULL))
    {
        return;
    }

    memset(packet, 0, TELEMETRY_PACKET_SIZE);

    packet[0] = TELEMETRY_MAGIC_HIGH;
    packet[1] = TELEMETRY_MAGIC_LOW;
    packet[2] = TELEMETRY_VERSION;
    packet[3] = TELEMETRY_MESSAGE_DATA;
    packet[4] = sequence;
    packet[5] = node_id;
    packet[6] = (uint8_t)(digital_input_mask & 0x0FU);
    packet[7] = sensor_data->valid_mask;

    Telemetry_WriteU16LE(&packet[8],
                         sensor_data->wind_speed_x10);

    Telemetry_WriteU16LE(&packet[10],
                         sensor_data->rainfall_x10);

    Telemetry_WriteU16LE(&packet[12],
                         (uint16_t)sensor_data->soil_temperature_x10);

    Telemetry_WriteU16LE(&packet[14],
                         sensor_data->soil_moisture_x10);

    Telemetry_WriteU16LE(&packet[16],
                         sensor_data->soil_ec_us_cm);

    Telemetry_WriteU16LE(&packet[18],
                         sensor_data->soil_ph_x100);

    Telemetry_WriteU16LE(&packet[20],
                         sensor_data->soil_n_mg_kg);

    Telemetry_WriteU16LE(&packet[22],
                         sensor_data->soil_p_mg_kg);

    Telemetry_WriteU16LE(&packet[24],
                         sensor_data->soil_k_mg_kg);

    packet[26] = (uint8_t)sensor_data->wind_status;
    packet[27] = (uint8_t)sensor_data->rain_status;
    packet[28] = (uint8_t)sensor_data->soil_status;
    packet[29] = 0U;

    crc = Modbus_CRC16(packet, 30U);
    packet[30] = (uint8_t)(crc & 0xFFU);
    packet[31] = (uint8_t)(crc >> 8U);
}
