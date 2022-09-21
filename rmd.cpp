#include "rmd.h"

RMD::RMD(int8_t frame_size, uint16_t motor_id, uint8_t reduction_ratio)
{
    _frame_size = frame_size;
    _motor_id = motor_id;
    _reduction_ratio = reduction_ratio;
}

void RMD::set_buf_for_read_cmd(uint8_t *buffer, uint8_t len, uint8_t cmd)
{
    if (!_does_equal_frame_size(len))
        return;
    buffer[0] = cmd;
    for (int i = 1; i < _frame_size; ++i)
    {
        buffer[i] = 0x00;
    }
    _frame_count++;
    return;
}

void RMD::print_data(char delim)
{
    // Serial.print("Temperature: ");
    // Serial.print(_motor_temperature);
    // Serial.print(delim);
    // Serial.print("Voltage: ");
    // Serial.print(_voltage);
    // Serial.print(delim);
    // Serial.print("Current: ");
    // Serial.print(_motor_torque_current_amps);
    // Serial.print(delim);
    // Serial.print("Speed: ");
    // Serial.print(_motor_speed);
    // Serial.print(delim);
    Serial.print("Angle: ");
    Serial.print(_multiturn_angle);
    Serial.print(delim);
    Serial.print("Position: ");
    Serial.print(_shaft_multiturn_position);
    Serial.print(delim);
    // Serial.print("Offset: ");
    // Serial.print(_encoder_offset);
    // Serial.print(delim);
    // Serial.print("Phase a current: ");
    // Serial.print(_phase_a_current);
    // Serial.print(delim);
    // Serial.print("Phase b current: ");
    // Serial.print(_phase_b_current);
    // Serial.print(delim);
    // Serial.print("Phase c current: ");
    // Serial.print(_phase_c_current);
    // Serial.print(delim);
    // Serial.print("Current loop KP: ");
    // Serial.print(_current_loop_kp, 5);
    // Serial.print(delim);
    // Serial.print("Current loop KI: ");
    // Serial.print(_current_loop_ki, 5);
    // Serial.print(delim);
    // Serial.print("Speed loop KP: ");
    // Serial.print(_speed_loop_kp, 5);
    // Serial.print(delim);
    // Serial.print("Speed loop KI: ");
    // Serial.print(_speed_loop_ki, 5);
    // Serial.print(delim);
    // Serial.print("Position loop KP: ");
    // Serial.print(_position_loop_kp, 5);
    // Serial.print(delim);
    // Serial.print("Position loop KI: ");
    // Serial.print(_position_loop_ki, 5);
    // Serial.print(delim);
    Serial.print("Error: ");
    Serial.println(_error_state);
}
/// @brief Parses reply from canbus, expecting array equal to frame size of the motor.
/// @param arr Array of frame size bytes from can bus.
/// @param len Expects frame_size (likely 8)
void RMD::parse_reply(uint8_t *arr, int8_t len)
{
    if (!_does_equal_frame_size(len))
        return;
    _frame_count--;
    switch (arr[0])
    {
    case READ_MOTOR_STATUS_1:
        _read_motor_status_1(arr);
        break;
    case READ_MOTOR_STATUS_2:
        _read_motor_status_2(arr);
        break;
    case READ_MOTOR_STATUS_3:
        _read_motor_status_3(arr);
        break;
    case READ_MULTITURN_ENCODER_POSITION:
        _read_multiturn_position(arr);
        break;
    case READ_MULTITURN_ENCODER_POSITION_OFFSET:
        _read_encoder_offset(arr);
        break;
    default:
        Serial.print(arr[0], HEX);
        Serial.println(" unrecognized");
        _frame_count--;
        break;
    }
}

/// @brief  Reads in motor temperature, voltage, and error state
///
/// @param arr 8 length array of type uint8_t
void RMD::_read_motor_status_1(uint8_t *arr)
{
    _voltage = ((float)(_combine_two_bytes_uint_16t(arr[5], arr[4]))) * 0.1;
    _motor_temperature = arr[1];
    _error_state = _combine_two_bytes_uint_16t(arr[7], arr[6]);
    return;
}

/// @brief This command reads the temperature, speed and encoder position of the current motor.
///         1. Motor temperature（int8_t type,unit 1℃/LSB）
///         2. Motor torque current(Iq)（int16_t type,Range:-2048~2048,real torque current range:-33A~33A）
///         3. Motor speed（int16_t type,1dps/LSB）
///         4. Encoder position value（uint16_t type,14bit encoder value range 0~16383）
/// @param arr _frame_size length array of type uint8_t
void RMD::_read_motor_status_2(uint8_t *arr)
{
    _motor_temperature = (int8_t)arr[1];
    _motor_torque_current_amps = ((float)_combine_two_bytes_int_16t(arr[3], arr[2])) * 0.01;
    _motor_speed = _combine_two_bytes_int_16t(arr[5], arr[4]) / _reduction_ratio;
    _encoder_position = _combine_two_bytes_uint_16t(arr[7], arr[6]);
    return;
}

void RMD::_read_motor_status_3(uint8_t *arr)
{
    _motor_temperature = (int8_t)arr[1];
    _phase_a_current = ((float)_combine_two_bytes_int_16t(arr[3], arr[2])) * 0.01;
    _phase_b_current = ((float)_combine_two_bytes_int_16t(arr[5], arr[4])) * 0.01;
    _phase_c_current = ((float)_combine_two_bytes_int_16t(arr[7], arr[6])) * 0.01;
    return;
}

void RMD::_read_multiturn_position(uint8_t *arr)
{
    uint8_t dir = arr[7];
    // combine arr's 4 bytes into a single int, little-endian.
    _motor_multiturn_position = (int32_t)arr[4] << 24 | (int32_t)arr[3] << 16 | (int32_t)arr[2] << 8 | (int32_t)arr[1];
    if (dir == 1)
        _motor_multiturn_position *= -1;
    _shaft_multiturn_position = _motor_multiturn_position / _reduction_ratio;
    _multiturn_angle = _shaft_multiturn_position / _encoder_resolution * 360.0;
}

void RMD::_read_encoder_offset(uint8_t *arr)
{
    union
    {
        int32_t val;
        uint8_t bytes[4];
    } position;

    position.bytes[3] = arr[0];
    position.bytes[2] = arr[1];
    position.bytes[1] = arr[2];
    position.bytes[0] = arr[3];
    _encoder_offset = position.val;
}

void RMD::_read_pid(uint8_t *arr)
{
    _current_loop_kp = (double)arr[2];
    _current_loop_ki = (double)arr[3];
    _speed_loop_kp = (double)arr[4];
    _speed_loop_ki = (double)arr[5];
    _position_loop_kp = (double)arr[6];
    _position_loop_ki = (double)arr[7];
    return;
}

void RMD::_write_pid_to_rom_reply(uint8_t *arr)
{
    for (int i = 0; i < 8; i++)
    {
        Serial.print(arr[i], HEX);
        Serial.print('\t');
    }
    Serial.println();
    _current_loop_kp = (double)arr[2];
    _current_loop_ki = (double)arr[3];
    _speed_loop_kp = (double)arr[4];
    _speed_loop_ki = (double)arr[5];
    _position_loop_kp = (double)arr[6];
    _position_loop_ki = (double)arr[7];
    return;
}

uint16_t RMD::_combine_two_bytes_uint_16t(uint8_t low, uint8_t high)
{
    return ((low << 8) | high);
}

int16_t RMD::_combine_two_bytes_int_16t(uint8_t low, uint8_t high)
{
    return ((low << 8) | high);
}

bool RMD::_does_equal_frame_size(int8_t len)
{
    return (len == _frame_size);
}

/// @brief The host sends this command to control the position of the motor (multi turn angle).
/// @param arr byte array of length _frame_size;
/// @param _angle multi-turn angle position in degrees
/// @param _rpm 0 - 1000;
void RMD::set_multiturn_position(uint8_t *arr, float _angle, uint16_t _rpm)
{
    union
    {
        uint32_t val;
        int8_t bytes[4];
    } angle;

    angle.val = _angle * 100 * _reduction_ratio;

    union
    {
        uint16_t val;
        uint8_t bytes[2];
    } dps;

    dps.val = _rpm_to_dps(_rpm);

    arr[0] = SET_MULTITURN_POSITION;
    arr[1] = 0x00;
    arr[2] = dps.bytes[0];
    arr[3] = dps.bytes[1];
    arr[4] = angle.bytes[0];
    arr[5] = angle.bytes[1];
    arr[6] = angle.bytes[2];
    arr[7] = angle.bytes[3];
    return;
}