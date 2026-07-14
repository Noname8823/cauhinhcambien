#include "station_app.h"

#include "rs485_port.h"
#include "modbus_master.h"
#include "Isolated_Input.h"

/* =========================================================
 * CẤU HÌNH CHUNG
 * ========================================================= */

/*
 * Lần đọc đầu tiên + thử lại 2 lần
 * = tối đa 3 lần đọc mỗi cảm biến.
 */
#define SENSOR_RETRY_COUNT          2U

/*
 * Khoảng nghỉ giữa hai cảm biến Modbus.
 */
#define SENSOR_GAP_MS               20U

/*
 * Chu kỳ đọc cảm biến và gửi dữ liệu.
 */
#define SEND_PERIOD_MS              1000U

/*
 * 1U: đọc cảm biến Modbus thật.
 * 0U: không đọc cảm biến, gửi giá trị 0.
 */
#define ENABLE_SENSOR_POLLING       1U

/* =========================================================
 * ĐỊNH DẠNG GÓI TIN
 * ========================================================= */

/*
 * Hai byte nhận diện đầu gói.
 */
#define PACKET_HEADER_0             0xA5U
#define PACKET_HEADER_1             0x5AU

/*
 * Mỗi cảm biến trong gói chiếm:
 *
 * Slave ID    : 1 byte
 * Value       : 2 byte
 *
 * Tổng        : 3 byte
 */
#define SENSOR_RECORD_SIZE          3U

/*
 * Phần cố định của gói:
 *
 * Header          : 2 byte
 * Length          : 1 byte
 * Digital Input   : 1 byte
 * CRC             : 2 byte
 *
 * Tổng            : 6 byte
 */
#define PACKET_FIXED_SIZE           6U

/*
 * Mỗi cảm biến chỉ đọc đúng một thanh ghi.
 */
#define SENSOR_REGISTER_COUNT       1U

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
     * Function code:
     *
     * 0x03 = Read Holding Registers
     * 0x04 = Read Input Registers
     */
    uint8_t function_code;

    /*
     * Địa chỉ thanh ghi cần đọc.
     */
    uint16_t register_address;

} SensorConfig_t;

/* =========================================================
 * DỮ LIỆU RUNTIME CỦA CẢM BIẾN
 * ========================================================= */

typedef struct
{
    /*
     * Trạng thái lần đọc gần nhất.
     *
     * Trạng thái chỉ dùng nội bộ,
     * không được đưa vào gói tin.
     */
    ModbusStatus_t status;

    /*
     * Giá trị raw 16-bit của cảm biến.
     */
    uint16_t value;

} SensorRuntime_t;

/* =========================================================
 * DANH SÁCH CẢM BIẾN
 * ========================================================= */

/*
 * Cấu trúc mỗi dòng:
 *
 * {
 *     Slave ID,
 *     Function code,
 *     Register address
 * }
 *
 * Mỗi cảm biến chỉ lấy một thanh ghi.
 *
 * Cảm biến đất ID 3:
 * chỉ lấy register 0 là nhiệt độ đất.
 */
static const SensorConfig_t sensor_list[] =
{
    /*
     * Cảm biến gió:
     *
     * Slave ID        = 1
     * Function code   = 0x03
     * Register        = 0x0000
     */
    {
        1U,
        0x03U,
        0x0000U
    },

    /*
     * Cảm biến mưa:
     *
     * Slave ID        = 2
     * Function code   = 0x03
     * Register        = 0x0000
     */
    {
        2U,
        0x03U,
        0x0000U
    },

    /*
     * Cảm biến đất:
     *
     * Slave ID        = 3
     * Function code   = 0x03
     * Register        = 0x0000
     *
     * Chỉ lấy nhiệt độ đất.
     */
    {
        3U,
        0x03U,
        0x0000U
    }
};

/* =========================================================
 * TÍNH TỰ ĐỘNG SỐ CẢM BIẾN VÀ KÍCH THƯỚC GÓI
 * ========================================================= */

/*
 * Tự động đếm số phần tử trong sensor_list[].
 */
#define SENSOR_COUNT                                           \
    ((uint16_t)(sizeof(sensor_list) / sizeof(sensor_list[0])))

/*
 * Tổng kích thước gói:
 *
 * 6 byte cố định
 * + số cảm biến × 3 byte.
 *
 * 3 cảm biến:
 * 6 + 3 × 3 = 15 byte = 0x0F.
 *
 * 4 cảm biến:
 * 6 + 4 × 3 = 18 byte = 0x12.
 */
#define PACKET_TOTAL_SIZE                                      \
    ((uint16_t)(PACKET_FIXED_SIZE +                            \
                (SENSOR_COUNT * SENSOR_RECORD_SIZE)))

/*
 * Length chỉ có một byte.
 * Gói không được lớn hơn 255 byte.
 */
_Static_assert(
    PACKET_TOTAL_SIZE <= 255U,
    "Packet length exceeds one-byte length field"
);

/* =========================================================
 * CỔNG RS485
 * ========================================================= */

/*
 * USART1 + MAX485 số 1:
 * đọc cảm biến Modbus.
 */
static RS485_Port_t sensor_rs485;

/*
 * USART2 + MAX485 số 2:
 * gửi gói sang E32 hoặc USB-RS485.
 */
static RS485_Port_t e32_rs485;

/*
 * Dữ liệu runtime tương ứng từng cảm biến.
 */
static SensorRuntime_t sensor_runtime[SENSOR_COUNT];

/*
 * Thời điểm gửi gần nhất.
 */
static uint32_t previous_send_tick = 0U;

/* =========================================================
 * XÓA DỮ LIỆU CẢM BIẾN
 * ========================================================= */

static void ClearSensorRuntime(void)
{
    uint16_t sensor_index;

    for (sensor_index = 0U;
         sensor_index < SENSOR_COUNT;
         sensor_index++)
    {
        sensor_runtime[sensor_index].status =
            MODBUS_STATUS_TIMEOUT;

        sensor_runtime[sensor_index].value =
            0U;
    }
}

/* =========================================================
 * ĐỌC TẤT CẢ CẢM BIẾN
 * ========================================================= */

static void ReadAllSensors(void)
{
#if (ENABLE_SENSOR_POLLING == 0U)

    /*
     * Chế độ chỉ test Digital Input.
     * Không gửi yêu cầu Modbus.
     */
    ClearSensorRuntime();

#else

    uint16_t sensor_index;

    for (sensor_index = 0U;
         sensor_index < SENSOR_COUNT;
         sensor_index++)
    {
        /*
         * Xóa giá trị cũ.
         *
         * Nếu đọc lỗi, giá trị gửi đi sẽ là 0x0000.
         */
        sensor_runtime[sensor_index].value =
            0U;

        /*
         * Mỗi cảm biến chỉ đọc một thanh ghi.
         */
        sensor_runtime[sensor_index].status =
            Modbus_ReadRegisters(
                &sensor_rs485,
                sensor_list[sensor_index].slave_id,
                sensor_list[sensor_index].function_code,
                sensor_list[sensor_index].register_address,
                SENSOR_REGISTER_COUNT,
                &sensor_runtime[sensor_index].value,
                SENSOR_RETRY_COUNT
            );

        /*
         * Nếu timeout, CRC lỗi hoặc response lỗi,
         * không dùng dữ liệu cũ.
         */
        if (sensor_runtime[sensor_index].status !=
            MODBUS_STATUS_OK)
        {
            sensor_runtime[sensor_index].value =
                0U;
        }

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
 * THÊM MỘT CẢM BIẾN VÀO GÓI
 * ========================================================= */

/*
 * Cấu trúc một record:
 *
 * Byte 0: Slave ID
 * Byte 1: Value high
 * Byte 2: Value low
 */
static uint8_t Packet_AddSensorRecord(
    uint8_t *packet,
    uint16_t packet_size,
    uint16_t *offset,
    const SensorConfig_t *config,
    const SensorRuntime_t *runtime)
{
    uint16_t value;

    if ((packet == NULL) ||
        (offset == NULL) ||
        (config == NULL) ||
        (runtime == NULL))
    {
        return 0U;
    }

    /*
     * Kiểm tra tránh ghi vượt bộ đệm.
     */
    if ((*offset + SENSOR_RECORD_SIZE) >
        packet_size)
    {
        return 0U;
    }

    value = runtime->value;

    /*
     * Slave ID.
     */
    packet[*offset] =
        config->slave_id;

    (*offset)++;

    /*
     * Value high.
     */
    packet[*offset] =
        (uint8_t)(value >> 8U);

    (*offset)++;

    /*
     * Value low.
     */
    packet[*offset] =
        (uint8_t)(value & 0x00FFU);

    (*offset)++;

    return 1U;
}

/* =========================================================
 * TẠO GÓI BINARY
 * ========================================================= */

/*
 * Cấu trúc gói:
 *
 * Byte 0: Header 0 = A5
 * Byte 1: Header 1 = 5A
 * Byte 2: Tổng chiều dài gói
 * Byte 3: Digital Input mask
 *
 * Byte 4...:
 *
 * [Slave ID][Value high][Value low]
 * [Slave ID][Value high][Value low]
 * ...
 *
 * Hai byte cuối:
 *
 * CRC low
 * CRC high
 */
static uint16_t BuildRawPacket(
    uint8_t *packet,
    uint16_t packet_size,
    uint8_t digital_input_mask)
{
    uint16_t offset;
    uint16_t crc;
    uint16_t sensor_index;

    if ((packet == NULL) ||
        (packet_size < PACKET_TOTAL_SIZE))
    {
        return 0U;
    }

    /*
     * Byte 0-1: Header.
     */
    packet[0] = PACKET_HEADER_0;
    packet[1] = PACKET_HEADER_1;

    /*
     * Byte 2: tổng chiều dài toàn bộ gói,
     * bao gồm hai byte CRC.
     */
    packet[2] =
        (uint8_t)PACKET_TOTAL_SIZE;

    /*
     * Byte 3: Digital Input.
     *
     * bit0 = IN1
     * bit1 = IN2
     * bit2 = IN3
     * bit3 = IN4
     */
    packet[3] =
        digital_input_mask & 0x0FU;

    /*
     * Record đầu tiên bắt đầu từ byte 4.
     */
    offset = 4U;

    /*
     * Thêm toàn bộ cảm biến vào gói.
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
     * Trước khi thêm CRC:
     *
     * offset phải bằng:
     * 4 + SENSOR_COUNT × 3.
     *
     * Phải còn đúng 2 byte dành cho CRC.
     */
    if ((offset + 2U) != PACKET_TOTAL_SIZE)
    {
        return 0U;
    }

    /*
     * Tính CRC trên toàn bộ dữ liệu
     * từ Header đến record cảm biến cuối cùng.
     */
    crc = Modbus_CalculateCRC16(
        packet,
        offset
    );

    /*
     * CRC Modbus:
     * byte thấp trước.
     */
    packet[offset] =
        (uint8_t)(crc & 0x00FFU);

    offset++;

    /*
     * CRC byte cao.
     */
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
    uint8_t packet[PACKET_TOTAL_SIZE];
    uint16_t packet_length;

    /*
     * Tạo gói binary.
     */
    packet_length =
        BuildRawPacket(
            packet,
            sizeof(packet),
            digital_input_mask
        );

    /*
     * Chỉ gửi khi tạo đúng đủ gói.
     */
    if (packet_length != PACKET_TOTAL_SIZE)
    {
        return;
    }

    /*
     * USART2
     * → MAX485 số 2
     * → E32 hoặc USB-RS485.
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
     * Kiểm tra con trỏ UART.
     */
    if ((sensor_uart == NULL) ||
        (e32_uart == NULL))
    {
        return;
    }

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
     * gửi gói binary.
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
     * Xóa dữ liệu ban đầu.
     */
    ClearSensorRuntime();

    previous_send_tick =
        HAL_GetTick();
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
     * Chưa đủ chu kỳ gửi thì thoát.
     */
    if ((current_tick - previous_send_tick) <
        SEND_PERIOD_MS)
    {
        return;
    }

    previous_send_tick =
        current_tick;

    /*
     * 1. Đọc các cảm biến qua USART1.
     */
    ReadAllSensors();

    /*
     * 2. Đọc bốn Digital Input.
     */
    digital_input_mask =
        Input_GetActiveMask();

    /*
     * 3. Tạo và gửi gói qua USART2.
     */
    SendRawPacketToE32(
        digital_input_mask
    );

    /*
     * Đảo LED báo chương trình còn chạy.
     */
    HAL_GPIO_TogglePin(
        LED_GPIO_Port,
        LED_Pin
    );
}
