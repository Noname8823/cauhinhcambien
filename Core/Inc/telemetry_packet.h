#ifndef INC_TELEMETRY_PACKET_H_
#define INC_TELEMETRY_PACKET_H_

#include "station_sensors.h"
#include <stdint.h>

#define TELEMETRY_PACKET_SIZE       32U
#define TELEMETRY_MAGIC_HIGH        0xA5U
#define TELEMETRY_MAGIC_LOW         0x5AU
#define TELEMETRY_VERSION           0x01U
#define TELEMETRY_MESSAGE_DATA      0x01U

/*
 * Goi tin co dinh 32 byte gui den E32 RS485.
 *
 *  0       Magic high = A5
 *  1       Magic low  = 5A
 *  2       Version
 *  3       Message type
 *  4       Sequence
 *  5       Node ID
 *  6       DI active mask: bit0..bit3 = DI1..DI4
 *  7       Sensor valid mask: bit0 wind, bit1 rain, bit2 soil
 *  8..9    Wind speed x10, little-endian
 * 10..11   Rainfall x10, little-endian
 * 12..13   Soil temperature x10, signed int16, little-endian
 * 14..15   Soil moisture x10
 * 16..17   Soil EC uS/cm
 * 18..19   Soil pH x100
 * 20..21   Soil N mg/kg
 * 22..23   Soil P mg/kg
 * 24..25   Soil K mg/kg
 * 26       Wind Modbus status
 * 27       Rain Modbus status
 * 28       Soil Modbus status
 * 29       Reserved
 * 30..31   CRC16-Modbus of byte 0..29, CRC low first
 */

void Telemetry_BuildPacket(uint8_t packet[TELEMETRY_PACKET_SIZE],
                           uint8_t sequence,
                           uint8_t node_id,
                           uint8_t digital_input_mask,
                           const StationSensorData_t *sensor_data);

#endif
