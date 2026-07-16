#include "at_command.h"

#include "modbus_master.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AT_LINE_SIZE          128U
#define AT_RESPONSE_SIZE      192U
#define AT_SENSOR_RETRY       2U

static RS485_Port_t *at_bus;
static AppConfig_t *at_config;

static AT_InputMaskReader_t
    at_input_reader;

static char at_line[
    AT_LINE_SIZE];

static uint16_t at_line_length;

static void AT_SendText(
    const char *text)
{
    if ((at_bus == NULL) ||
        (text == NULL))
    {
        return;
    }

    (void)RS485_Send(
        at_bus,
        (const uint8_t *)text,
        strlen(text),
        1000U);
}

static void AT_SendFormatted(
    const char *format,
    ...)
{
    char response[
        AT_RESPONSE_SIZE];

    va_list arguments;
    int length;

    va_start(
        arguments,
        format);

    length =
        vsnprintf(
            response,
            sizeof(response),
            format,
            arguments);

    va_end(arguments);

    if (length <= 0)
    {
        return;
    }

    response[
        sizeof(response) - 1U] =
        '\0';

    AT_SendText(response);
}

static void AT_SendOK(void)
{
    AT_SendText("OK\r\n");
}

static void AT_SendError(
    const char *error)
{
    if (error == NULL)
    {
        AT_SendText("ERR\r\n");
        return;
    }

    AT_SendFormatted(
        "ERR:%s\r\n",
        error);
}

static uint8_t AT_ParseUnsigned(
    const char *text,
    uint32_t minimum,
    uint32_t maximum,
    uint32_t *value)
{
    char *end_pointer;
    unsigned long parsed;

    if ((text == NULL) ||
        (value == NULL) ||
        (*text == '\0'))
    {
        return 0U;
    }

    end_pointer = NULL;

    parsed =
        strtoul(
            text,
            &end_pointer,
            0);

    if ((end_pointer == text) ||
        (*end_pointer != '\0'))
    {
        return 0U;
    }

    if ((parsed < minimum) ||
        (parsed > maximum))
    {
        return 0U;
    }

    *value = parsed;

    return 1U;
}

static uint8_t AT_ParseSigned(
    const char *text,
    int32_t minimum,
    int32_t maximum,
    int32_t *value)
{
    char *end_pointer;
    long parsed;

    if ((text == NULL) ||
        (value == NULL) ||
        (*text == '\0'))
    {
        return 0U;
    }

    end_pointer = NULL;

    parsed =
        strtol(
            text,
            &end_pointer,
            0);

    if ((end_pointer == text) ||
        (*end_pointer != '\0'))
    {
        return 0U;
    }

    if ((parsed < minimum) ||
        (parsed > maximum))
    {
        return 0U;
    }

    *value = parsed;

    return 1U;
}

static uint8_t AT_SplitArguments(
    const char *arguments,
    char *storage,
    uint16_t storage_size,
    char **tokens,
    uint8_t maximum_tokens)
{
    char *token;
    uint8_t count;

    if ((arguments == NULL) ||
        (storage == NULL) ||
        (tokens == NULL))
    {
        return 0U;
    }

    strncpy(
        storage,
        arguments,
        storage_size - 1U);

    storage[
        storage_size - 1U] =
        '\0';

    count = 0U;

    token =
        strtok(storage, ",");

    while ((token != NULL) &&
           (count <
            maximum_tokens))
    {
        tokens[count++] = token;

        token =
            strtok(NULL, ",");
    }

    if (token != NULL)
    {
        return 0U;
    }

    return count;
}

static uint8_t AT_ValidateSensor(
    uint32_t slave_id,
    uint32_t function_code,
    uint32_t register_count)
{
    if ((slave_id < 1U) ||
        (slave_id >
         APP_MAX_SENSORS))
    {
        return 0U;
    }

    if ((function_code != 3U) &&
        (function_code != 4U))
    {
        return 0U;
    }

    if ((register_count < 1U) ||
        (register_count >
         APP_MAX_REGISTERS))
    {
        return 0U;
    }

    return 1U;
}

static void AT_GetConfig(void)
{
    uint8_t index;

    const char *role_text;

    role_text =
        (at_config->role ==
         APP_ROLE_RX)
            ? "RX"
            : "TX";

    AT_SendFormatted(
        "CFG ID=%u DST=%u ROLE=%s "
        "FREQ=%lu BW=%u SF=%u "
        "CR=%u PWR=%d\r\n",
        at_config->node_id,
        at_config->destination_id,
        role_text,
        (unsigned long)
            at_config->frequency,
        at_config->bandwidth,
        at_config->spreading_factor,
        at_config->coding_rate,
        (int)at_config->tx_power);

    for (index = 0U;
         index < APP_MAX_SENSORS;
         index++)
    {
        AppSensorConfig_t *sensor =
            &at_config->
                sensors[index];

        AT_SendFormatted(
            "SENSOR IDX=%u EN=%u "
            "SID=%u FC=%u "
            "REG=0x%04X CNT=%u\r\n",
            index,
            sensor->enabled,
            sensor->slave_id,
            sensor->function_code,
            sensor->start_register,
            sensor->register_count);
    }

    AT_SendText("END\r\n");
}

static void AT_TestSensor(
    const char *arguments)
{
    char storage[96];
    char *tokens[4];

    uint8_t token_count;

    uint32_t slave_id;
    uint32_t function_code;
    uint32_t start_register;
    uint32_t register_count;

    uint16_t values[
        APP_MAX_REGISTERS];

    ModbusStatus_t status;

    token_count =
        AT_SplitArguments(
            arguments,
            storage,
            sizeof(storage),
            tokens,
            4U);

    if (token_count != 4U)
    {
        AT_SendText(
            "SENSORTEST ERR "
            "STATUS=FORMAT\r\n");

        return;
    }

    if ((AT_ParseUnsigned(
             tokens[0],
             1U,
             APP_MAX_SENSORS,
             &slave_id) == 0U) ||

        (AT_ParseUnsigned(
             tokens[1],
             3U,
             4U,
             &function_code) == 0U) ||

        (AT_ParseUnsigned(
             tokens[2],
             0U,
             0xFFFFU,
             &start_register) == 0U) ||

        (AT_ParseUnsigned(
             tokens[3],
             1U,
             APP_MAX_REGISTERS,
             &register_count) == 0U))
    {
        AT_SendText(
            "SENSORTEST ERR "
            "STATUS=VALUE\r\n");

        return;
    }

    if (AT_ValidateSensor(
            slave_id,
            function_code,
            register_count) == 0U)
    {
        AT_SendText(
            "SENSORTEST ERR "
            "STATUS=VALUE\r\n");

        return;
    }

    memset(
        values,
        0,
        sizeof(values));

    /*
     * USB-RS485 nghe chung bus nên sẽ thấy
     * frame Modbus nhị phân.
     *
     * Gửi CRLF để tách dữ liệu nhị phân
     * khỏi dòng SENSORTEST.
     */
    AT_SendText("\r\n");

    status =
        Modbus_ReadRegisters(
            at_bus,
            slave_id,
            function_code,
            start_register,
            register_count,
            values,
            AT_SENSOR_RETRY);

    /*
     * Kết thúc phần dữ liệu nhị phân
     * mà USB-RS485 có thể nhìn thấy.
     */
    AT_SendText("\r\n");

    if (status ==
        MODBUS_STATUS_OK)
    {
        AT_SendFormatted(
            "SENSORTEST OK SID=%lu "
            "FC=%lu REG=0x%04lX "
            "CNT=%lu FIRST=0x%04X\r\n",
            (unsigned long)slave_id,
            (unsigned long)function_code,
            (unsigned long)start_register,
            (unsigned long)register_count,
            values[0]);
    }
    else
    {
        AT_SendFormatted(
            "SENSORTEST ERR "
            "STATUS=%u\r\n",
            (unsigned int)status);
    }
}

static void AT_SetSensor(
    const char *arguments)
{
    char storage[128];
    char *tokens[6];

    uint8_t token_count;

    uint32_t index;
    uint32_t enabled;
    uint32_t slave_id;
    uint32_t function_code;
    uint32_t start_register;
    uint32_t register_count;

    AppSensorConfig_t sensor;

    token_count =
        AT_SplitArguments(
            arguments,
            storage,
            sizeof(storage),
            tokens,
            6U);

    if (token_count != 6U)
    {
        AT_SendError("FORMAT");
        return;
    }

    if ((AT_ParseUnsigned(
             tokens[0],
             0U,
             APP_MAX_SENSORS - 1U,
             &index) == 0U) ||

        (AT_ParseUnsigned(
             tokens[1],
             0U,
             1U,
             &enabled) == 0U) ||

        (AT_ParseUnsigned(
             tokens[2],
             1U,
             APP_MAX_SENSORS,
             &slave_id) == 0U) ||

        (AT_ParseUnsigned(
             tokens[3],
             3U,
             4U,
             &function_code) == 0U) ||

        (AT_ParseUnsigned(
             tokens[4],
             0U,
             0xFFFFU,
             &start_register) == 0U) ||

        (AT_ParseUnsigned(
             tokens[5],
             1U,
             APP_MAX_REGISTERS,
             &register_count) == 0U))
    {
        AT_SendError("VALUE");
        return;
    }

    memset(
        &sensor,
        0,
        sizeof(sensor));

    sensor.enabled =
        enabled;

    sensor.slave_id =
        slave_id;

    sensor.function_code =
        function_code;

    sensor.start_register =
        start_register;

    sensor.register_count =
        register_count;

    at_config->sensors[index] =
        sensor;

    AT_SendOK();
}

static void AT_ProcessLine(
    char *line)
{
    uint32_t unsigned_value;
    int32_t signed_value;

    if ((line == NULL) ||
        (*line == '\0'))
    {
        return;
    }

    if (strcmp(line, "AT") == 0)
    {
        AT_SendOK();
    }
    else if (strcmp(
                 line,
                 "AT+GETCFG") == 0)
    {
        AT_GetConfig();
    }
    else if (strcmp(
                 line,
                 "AT+SAVE") == 0)
    {
        if (AppConfig_Save() ==
            HAL_OK)
        {
            AT_SendOK();
        }
        else
        {
            AT_SendError("FLASH");
        }
    }
    else if (strcmp(
                 line,
                 "AT+GETIN") == 0)
    {
        uint8_t mask = 0U;

        if (at_input_reader != NULL)
        {
            mask =
                at_input_reader();
        }

        AT_SendFormatted(
            "INPUT MASK=0x%02X\r\n",
            mask);
    }
    else if (strncmp(
                 line,
                 "AT+TESTSENSOR=",
                 14U) == 0)
    {
        AT_TestSensor(
            line + 14U);
    }
    else if (strncmp(
                 line,
                 "AT+SENSOR=",
                 10U) == 0)
    {
        AT_SetSensor(
            line + 10U);
    }
    else if (strncmp(
                 line,
                 "AT+SETID=",
                 9U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 9U,
                0U,
                255U,
                &unsigned_value) == 0U)
        {
            AT_SendError("ID");
            return;
        }

        at_config->node_id =
            unsigned_value;

        at_config->role =
            (unsigned_value == 0U)
                ? APP_ROLE_RX
                : APP_ROLE_TX;

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETDST=",
                 10U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 10U,
                0U,
                255U,
                &unsigned_value) == 0U)
        {
            AT_SendError("DST");
            return;
        }

        at_config->destination_id =
            unsigned_value;

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETROLE=",
                 11U) == 0)
    {
        if (strcmp(
                line + 11U,
                "TX") == 0)
        {
            at_config->role =
                APP_ROLE_TX;
        }
        else if (strcmp(
                     line + 11U,
                     "RX") == 0)
        {
            at_config->role =
                APP_ROLE_RX;
        }
        else
        {
            AT_SendError("ROLE");
            return;
        }

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETFREQ=",
                 11U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 11U,
                100000000UL,
                1000000000UL,
                &unsigned_value) == 0U)
        {
            AT_SendError("FREQ");
            return;
        }

        switch (unsigned_value)
        {
            case 433000000UL:
            case 470000000UL:
            case 868000000UL:
            case 915000000UL:
            case 920000000UL:
                at_config->frequency =
                    unsigned_value;
                break;

            default:
                AT_SendError("FREQ");
                return;
        }

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETBW=",
                 9U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 9U,
                0U,
                2U,
                &unsigned_value) == 0U)
        {
            AT_SendError("BW");
            return;
        }

        at_config->bandwidth =
            unsigned_value;

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETSF=",
                 9U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 9U,
                7U,
                12U,
                &unsigned_value) == 0U)
        {
            AT_SendError("SF");
            return;
        }

        at_config->spreading_factor =
            unsigned_value;

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETCR=",
                 9U) == 0)
    {
        if (AT_ParseUnsigned(
                line + 9U,
                1U,
                4U,
                &unsigned_value) == 0U)
        {
            AT_SendError("CR");
            return;
        }

        at_config->coding_rate =
            unsigned_value;

        AT_SendOK();
    }
    else if (strncmp(
                 line,
                 "AT+SETPWR=",
                 10U) == 0)
    {
        if (AT_ParseSigned(
                line + 10U,
                -9,
                22,
                &signed_value) == 0U)
        {
            AT_SendError("PWR");
            return;
        }

        at_config->tx_power =
            signed_value;

        AT_SendOK();
    }
    else
    {
        AT_SendError("UNKNOWN");
    }
}

void AT_CommandInit(
    RS485_Port_t *shared_uart1_bus,
    AppConfig_t *config,
    AT_InputMaskReader_t input_reader)
{
    at_bus =
        shared_uart1_bus;

    at_config = config;

    at_input_reader =
        input_reader;

    at_line_length = 0U;

    memset(
        at_line,
        0,
        sizeof(at_line));
}

void AT_CommandNotifyReady(void)
{
    AT_SendText(
        "READY CONFIG\r\n");
}

void AT_CommandTask(void)
{
    uint8_t received_byte;
    HAL_StatusTypeDef status;

    if ((at_bus == NULL) ||
        (at_bus->uart == NULL) ||
        (at_config == NULL))
    {
        return;
    }

    RS485_SetReceiveMode(at_bus);

    status =
        HAL_UART_Receive(
            at_bus->uart,
            &received_byte,
            1U,
            2U);

    if (status != HAL_OK)
    {
        return;
    }

    if ((received_byte == '\r') ||
        (received_byte == '\n'))
    {
        if (at_line_length > 0U)
        {
            at_line[
                at_line_length] =
                '\0';

            AT_ProcessLine(at_line);

            at_line_length = 0U;
        }

        return;
    }

    if (at_line_length >=
        AT_LINE_SIZE - 1U)
    {
        at_line_length = 0U;

        AT_SendError(
            "LINE_TOO_LONG");

        return;
    }

    at_line[
        at_line_length++] =
        (char)received_byte;
}
