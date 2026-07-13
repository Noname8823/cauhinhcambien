#include "station_app.h"

#include "rs485_port.h"
#include "modbus_master.h"
#include "Isolated_Input.h"

#include <string.h>

/* =========================================================
 * CẤU HÌNH CHUNG
 * ========================================================= */

/*
 * retry_count = 2:
 *
 * Lần đọc đầu tiên
 * + thử lại 2 lần
 * = tối đa 3 lần.
 */
#define SENSOR_RETRY_COUNT          2U

/*
 * Khoảng nghỉ giữa hai cảm biến.
 */
#define SENSOR_GAP_MS               20U

/*
 * Chu kỳ đọc cảm biến và gửi dữ liệu.
 */
#define SEND_PERIOD_MS              1000U

/*
 * Số thanh ghi tối đa được lưu cho một cảm biến.
 */
#define MAX_SENSOR_REGISTERS        16U

/*
 * Kích thước tối đa của bộ đệm gửi E32.
 */
#define TX_PACKET_MAX_SIZE          128U

/*
 * 1U: đọc cảm biến Modbus bình thường.
 * 0U: không đọc cảm biến, chỉ test 4 Digital Input.
 */
#define ENABLE_SENSOR_POLLING       1U

/* =========================================================
 * HEADER GÓI TIN
 * ========================================================= */

#define PACKET_HEADER_0             0xA5U
#define PACKET_HEADER_1             0x5AU

/* =========================================================
 * LOẠI CẢM BIẾN
 * ========================================================= */

typedef enum
{
    SENSOR_TYPE_UNKNOWN      = 0x00U,
    SENSOR_TYPE_WIND         = 0x01U,
    SENSOR_TYPE_RAIN         = 0x02U,
    SENSOR_TYPE_SOIL         = 0x03U,
    SENSOR_TYPE_TEMPERATURE  = 0x04U,
    SENSOR_TYPE_HUMIDITY     = 0x05U

} SensorType_t;

/* =========================================================
 * CẤU HÌNH MỘT CẢM BIẾN
 * ========================================================= */

typedef struct
{
    /*
     * Địa chỉ Modbus Slave.
     */
    uint8_t slave_id;

    /*
     * Loại cảm biến.
     */
    uint8_t sensor_type;

    /*
     * Function code:
     *
     * 0x03 = Read Holding Registers
     * 0x04 = Read Input Registers
     */
    uint8_t function_code;

    /*
     * Địa chỉ thanh ghi bắt đầu.
     */
    uint16_t start_register;

    /*
     * Số thanh ghi cần đọc.
     */
    uint8_t register_count;

} SensorConfig_t;

/* =========================================================
 * DỮ LIỆU RUNTIME CỦA MỘT CẢM BIẾN
 * ========================================================= */

typedef struct
{
    /*
     * Trạng thái của lần đọc gần nhất.
     */
    ModbusStatus_t status;

    /*
     * Dữ liệu thanh ghi nguyên bản.
     *
     * Không chia 10.
     * Không chia 100.
     * Không đổi sang float.
     */
    uint16_t registers[MAX_SENSOR_REGISTERS];

} SensorRuntime_t;

/* =========================================================
 * DANH SÁCH CẢM BIẾN
 * ========================================================= */

/*
 * Cấu trúc mỗi phần tử:
 *
 * {
 *     slave_id,
 *     sensor_type,
 *     function_code,
 *     start_register,
 *     register_count
 * }
 */
static const SensorConfig_t sensor_list[] =
{
    /*
     * Cảm biến gió:
     *
     * Slave ID       = 1
     * Sensor type    = WIND
     * Function       = 0x03
     * Start register = 0x0000
     * Register count = 1
     */
    {
        1U,
        SENSOR_TYPE_WIND,
        0x03U,
        0x0000U,
        1U
    },

    /*
     * Cảm biến mưa:
     *
     * Slave ID       = 2
     * Sensor type    = RAIN
     * Function       = 0x03
     * Start register = 0x0000
     * Register count = 1
     */
    {
        2U,
        SENSOR_TYPE_RAIN,
        0x03U,
        0x0000U,
        1U
    },

    /*
     * Cảm biến đất:
     *
     * Slave ID       = 3
     * Sensor type    = SOIL
     * Function       = 0x03
     * Start register = 0x0000
     * Register count = 7
     */
    {
        3U,
        SENSOR_TYPE_SOIL,
        0x03U,
        0x0000U,
        7U
    }
};

#define SENSOR_COUNT                                           \
    ((uint8_t)(sizeof(sensor_list) / sizeof(sensor_list[0])))

/* =========================================================
 * CỔNG RS485
 * ========================================================= */

/*
 * USART1 + MAX485 số 1:
 * đọc bus cảm biến Modbus.
 */
static RS485_Port_t sensor_rs485;

/*
 * USART2 + MAX485 số 2:
 * gửi gói binary sang E32.
 */
static RS485_Port_t e32_rs485;

/*
 * Dữ liệu runtime tương ứng với sensor_list[].
 */
static SensorRuntime_t sensor_runtime[SENSOR_COUNT];

/*
 * Thời điểm bắt đầu chu kỳ gần nhất.
 */
static uint32_t previous_send_tick = 0U;

/*
 * Số thứ tự gói tin.
 */
static uint16_t sequence = 0U;

/* =========================================================
 * XÓA DỮ LIỆU RUNTIME
 * ========================================================= */

static void ClearSensorRuntime(void)
{
    uint8_t sensor_index;

    for (sensor_index = 0U;
         sensor_index < SENSOR_COUNT;
         sensor_index++)
    {
        memset(
            sensor_runtime[sensor_index].registers,
            0,
            sizeof(sensor_runtime[sensor_index].registers)
        );

        /*
         * Khi chưa đọc được cảm biến,
         * mặc định trạng thái là timeout.
         */
        sensor_runtime[sensor_index].status =
            MODBUS_STATUS_TIMEOUT;
    }
}

/* =========================================================
 * ĐỌC TẤT CẢ CẢM BIẾN
 * ========================================================= */

static void ReadAllSensors(void)
{
#if (ENABLE_SENSOR_POLLING == 0U)

    /*
     * Chỉ test Digital Input.
     * Không gửi yêu cầu Modbus.
     */
    ClearSensorRuntime();

#else

    uint8_t sensor_index;

    for (sensor_index = 0U;
         sensor_index < SENSOR_COUNT;
         sensor_index++)
    {
        /*
         * Xóa dữ liệu cũ của cảm biến.
         */
        memset(
            sensor_runtime[sensor_index].registers,
            0,
            sizeof(sensor_runtime[sensor_index].registers)
        );

        /*
         * Kiểm tra cấu hình số thanh ghi.
         */
        if ((sensor_list[sensor_index].register_count == 0U) ||
            (sensor_list[sensor_index].register_count >
             MAX_SENSOR_REGISTERS))
        {
            sensor_runtime[sensor_index].status =
                MODBUS_STATUS_BAD_PARAMETER;

            continue;
        }

        /*
         * Đọc nguyên các thanh ghi Modbus.
         */
        sensor_runtime[sensor_index].status =
            Modbus_ReadRegisters(
                &sensor_rs485,
                sensor_list[sensor_index].slave_id,
                sensor_list[sensor_index].function_code,
                sensor_list[sensor_index].start_register,
                sensor_list[sensor_index].register_count,
                sensor_runtime[sensor_index].registers,
                SENSOR_RETRY_COUNT
            );

        /*
         * Nghỉ giữa hai cảm biến.
         */
        if ((sensor_index + 1U) < SENSOR_COUNT)
        {
            HAL_Delay(SENSOR_GAP_MS);
        }
    }

#endif
}

/* =========================================================
 * THÊM MỘT RECORD CẢM BIẾN VÀO GÓI
 * ========================================================= */

/*
 * Cấu trúc một record:
 *
 * Byte 0: Slave ID
 * Byte 1: Sensor type
 * Byte 2: Modbus status
 * Byte 3: Register count
 * Byte 4...: dữ liệu thanh ghi raw
 *
 * Mỗi thanh ghi 16-bit:
 *
 * Byte cao trước
 * Byte thấp sau
 *
 * Ví dụ:
 *
 * Register = 0x02BC
 * Gửi      = 02 BC
 */
static uint8_t Packet_AddSensorRecord(
    uint8_t *packet,
    uint16_t packet_size,
    uint16_t *offset,
    const SensorConfig_t *config,
    const SensorRuntime_t *runtime)
{
    uint16_t required_size;
    uint16_t register_value;
    uint8_t register_index;

    if ((packet == NULL) ||
        (offset == NULL) ||
        (config == NULL) ||
        (runtime == NULL))
    {
        return 0U;
    }

    if ((config->register_count == 0U) ||
        (config->register_count >
         MAX_SENSOR_REGISTERS))
    {
        return 0U;
    }

    /*
     * 4 byte metadata:
     *
     * Slave ID
     * Sensor type
     * Status
     * Register count
     *
     * Mỗi register chiếm 2 byte.
     */
    required_size =
        4U +
        ((uint16_t)config->register_count * 2U);

    /*
     * Kiểm tra tránh ghi vượt packet[].
     */
    if ((*offset + required_size) >
        packet_size)
    {
        return 0U;
    }

    /*
     * Slave ID.
     */
    packet[*offset] =
        config->slave_id;

    (*offset)++;

    /*
     * Sensor type.
     */
    packet[*offset] =
        config->sensor_type;

    (*offset)++;

    /*
     * Trạng thái Modbus.
     */
    packet[*offset] =
        (uint8_t)runtime->status;

    (*offset)++;

    /*
     * Số thanh ghi.
     */
    packet[*offset] =
        config->register_count;

    (*offset)++;

    /*
     * Đưa dữ liệu thanh ghi raw vào gói.
     */
    for (register_index = 0U;
         register_index < config->register_count;
         register_index++)
    {
        register_value =
            runtime->registers[register_index];

        /*
         * Byte cao.
         */
        packet[*offset] =
            (uint8_t)(register_value >> 8U);

        (*offset)++;

        /*
         * Byte thấp.
         */
        packet[*offset] =
            (uint8_t)(register_value & 0x00FFU);

        (*offset)++;
    }

    return 1U;
}

/* =========================================================
 * TẠO GÓI RAW BINARY
 * ========================================================= */

/*
 * Cấu trúc gói mới:
 *
 * Byte 0: Header 0 = 0xA5
 * Byte 1: Header 1 = 0x5A
 * Byte 2: Tổng chiều dài gói
 * Byte 3: Sequence high
 * Byte 4: Sequence low
 * Byte 5: Digital Input mask
 * Byte 6: Số cảm biến
 *
 * Byte 7...:
 * Các record cảm biến
 *
 * Hai byte cuối:
 * CRC low
 * CRC high
 *
 * Không còn:
 *
 * Version
 * Message type
 */
static uint16_t BuildRawPacket(
    uint8_t *packet,
    uint16_t packet_size,
    uint8_t digital_input_mask)
{
    uint16_t offset;
    uint16_t total_length;
    uint16_t crc;
    uint8_t sensor_index;

    /*
     * Gói tối thiểu:
     *
     * Header       2 byte
     * Length       1 byte
     * Sequence     2 byte
     * DI           1 byte
     * Sensor count 1 byte
     * CRC          2 byte
     *
     * Tổng tối thiểu = 9 byte.
     */
    if ((packet == NULL) ||
        (packet_size < 9U))
    {
        return 0U;
    }

    /*
     * Byte 0-1: Header.
     */
    packet[0] = PACKET_HEADER_0;
    packet[1] = PACKET_HEADER_1;

    /*
     * Byte 2: tổng chiều dài.
     *
     * Chưa biết nên tạm thời đặt 0.
     */
    packet[2] = 0U;

    /*
     * Byte 3-4: sequence 16-bit.
     *
     * Byte cao trước.
     */
    packet[3] =
        (uint8_t)(sequence >> 8U);

    packet[4] =
        (uint8_t)(sequence & 0x00FFU);

    /*
     * Byte 5: Digital Input.
     *
     * bit0 = IN1
     * bit1 = IN2
     * bit2 = IN3
     * bit3 = IN4
     */
    packet[5] =
        digital_input_mask & 0x0FU;

    /*
     * Byte 6: số cảm biến.
     */
    packet[6] = SENSOR_COUNT;

    /*
     * Record đầu tiên bắt đầu từ byte 7.
     */
    offset = 7U;

    /*
     * Thêm lần lượt từng record cảm biến.
     */
    for (sensor_index = 0U;
         sensor_index < SENSOR_COUNT;
         sensor_index++)
    {
        if (Packet_AddSensorRecord(
                packet,
                packet_size,
                &offset,
                &sensor_list[sensor_index],
                &sensor_runtime[sensor_index]) == 0U)
        {
            return 0U;
        }
    }

    /*
     * Phía cuối cần thêm 2 byte CRC.
     */
    total_length = offset + 2U;

    /*
     * Trường length chỉ có 1 byte,
     * nên tổng gói không được vượt 255 byte.
     */
    if ((total_length > packet_size) ||
        (total_length > 255U))
    {
        return 0U;
    }

    /*
     * Điền tổng chiều dài vào byte 2.
     *
     * Chiều dài bao gồm cả hai byte CRC.
     */
    packet[2] =
        (uint8_t)total_length;

    /*
     * CRC tính trên toàn bộ gói
     * trước hai byte CRC.
     */
    crc = Modbus_CalculateCRC16(
        packet,
        offset
    );

    /*
     * CRC kiểu Modbus:
     *
     * Byte thấp trước.
     * Byte cao sau.
     */
    packet[offset] =
        (uint8_t)(crc & 0x00FFU);

    offset++;

    packet[offset] =
        (uint8_t)(crc >> 8U);

    offset++;

    return offset;
}

/* =========================================================
 * GỬI GÓI SANG E32
 * ========================================================= */

static void SendRawPacketToE32(
    uint8_t digital_input_mask)
{
    uint8_t packet[TX_PACKET_MAX_SIZE];
    uint16_t packet_length;

    /*
     * Xóa bộ đệm trước khi tạo gói.
     */
    memset(
        packet,
        0,
        sizeof(packet)
    );

    /*
     * Tạo gói raw.
     */
    packet_length =
        BuildRawPacket(
            packet,
            sizeof(packet),
            digital_input_mask
        );

    if (packet_length == 0U)
    {
        return;
    }

    /*
     * USART2
     * → MAX485 số 2
     * → E32.
     *
     * Chỉ gửi dữ liệu binary.
     */
    (void)RS485_Send(
        &e32_rs485,
        packet,
        packet_length,
        1000U
    );
}

/* =========================================================
 * KHỞI TẠO ỨNG DỤNG
 * ========================================================= */

void StationApp_Init(
    UART_HandleTypeDef *sensor_uart,
    UART_HandleTypeDef *e32_uart)
{
    /*
     * USART1 + MAX485 số 1:
     * đọc cảm biến Modbus.
     */
    RS485_PortInit(
        &sensor_rs485,
        sensor_uart,
        RS485_SENSOR_DE_GPIO_Port,
        RS485_SENSOR_DE_Pin,
        RS485_SENSOR_RE_GPIO_Port,
        RS485_SENSOR_RE_Pin
    );

    /*
     * USART2 + MAX485 số 2:
     * gửi gói binary sang E32.
     */
    RS485_PortInit(
        &e32_rs485,
        e32_uart,
        RS485_E32_DE_GPIO_Port,
        RS485_E32_DE_Pin,
        RS485_E32_RE_GPIO_Port,
        RS485_E32_RE_Pin
    );

    /*
     * Khởi tạo Digital Input.
     */
    Input_Init();

    /*
     * Xóa dữ liệu runtime.
     */
    ClearSensorRuntime();

    previous_send_tick =
        HAL_GetTick();

    sequence = 0U;

    /*
     * Không gửi chuỗi ASCII khởi động.
     */
}

/* =========================================================
 * TASK CHÍNH
 * ========================================================= */

void StationApp_Task(void)
{
    uint32_t current_tick;
    uint8_t digital_input_mask;

    current_tick =
        HAL_GetTick();

    /*
     * Chưa đủ thời gian một chu kỳ thì thoát.
     */
    if ((current_tick - previous_send_tick) <
        SEND_PERIOD_MS)
    {
        return;
    }

    previous_send_tick =
        current_tick;

    /*
     * 1. Đọc toàn bộ cảm biến qua USART1.
     */
    ReadAllSensors();

    /*
     * 2. Đọc bốn Digital Input.
     */
    digital_input_mask =
        Input_GetActiveMask();

    /*
     * 3. Đóng gói raw và gửi qua USART2.
     */
    SendRawPacketToE32(
        digital_input_mask
    );

    /*
     * Đảo LED báo chương trình còn hoạt động.
     */
    HAL_GPIO_TogglePin(
        LED_GPIO_Port,
        LED_Pin
    );

    /*
     * Tăng số thứ tự gói.
     */
    sequence++;
}
