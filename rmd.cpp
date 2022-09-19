#include "rmd.h"

RMD::RMD(int8_t frame_size = 8, uint16_t motor_id = 0x141)
{
    _frame_size = frame_size;
    _motor_id = motor_id;
}

void RMD::set_buf_for_read_cmd(uint8_t *buffer, uint8_t len, uint8_t cmd)
{
    if (!_does_equal_frame_size(len))
        return;
    buffer[0] = cmd;
    for (int i = 1; i < _frame_size; ++i)
    {
        buffer[1] = 0x00;
    }
    _frame_count++;
    return;
}

void RMD::print_data(char delim)
{
    Serial.print("Temperature: ");
    Serial.print(_motor_temperature);
    Serial.print(delim);
    Serial.print("Voltage: ");
    Serial.print(_voltage);
    Serial.print(delim);
    Serial.print("Current: ");
    Serial.print(_motor_torque_current_amps);
    Serial.print(delim);
    Serial.print("Speed: ");
    Serial.print(_motor_speed);
    Serial.print(delim);
    Serial.print("Encoder: ");
    Serial.print(_encoder_position);
    Serial.print(delim);
    Serial.print("Phase a current: ");
    Serial.print(_phase_a_current);
    Serial.print(delim);
    Serial.print("Phase b current: ");
    Serial.print(_phase_b_current);
    Serial.print(delim);
    Serial.print("Phase c current: ");
    Serial.print(_phase_c_current);
    Serial.print(delim);
    Serial.print("Current loop KP: ");
    Serial.print(_current_loop_kp);
    Serial.print(delim);
    Serial.print("Current loop KI: ");
    Serial.print(_current_loop_ki);
    Serial.print(delim);
    Serial.print("Speed loop KP: ");
    Serial.print(_speed_loop_kp);
    Serial.print(delim);
    Serial.print("Speed loop KI: ");
    Serial.print(_speed_loop_ki);
    Serial.print(delim);
    Serial.print("Position loop KP: ");
    Serial.print(_position_loop_kp);
    Serial.print(delim);
    Serial.print("Position loop KI: ");
    Serial.print(_position_loop_ki);
    Serial.print(delim);
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
    case READ_PID:
        _read_pid(arr);
        break;
    case WRITE_PID_TO_RAM:
        break;
    case WRITE_PID_TO_ROM:
        break;
    case READ_ACCELERATION:
        break;
    case WRITE_ACCELERATION_TO_RAM:
        break;
    case READ_ENCODER:
        break;
    case WRITE_ENCODER_OFFSET:
        break;
    case WRITE_CURRENT_POS_TO_ROM_AS_MOTOR_ZERO:
        break;
    case READ_MULTI_TURN_ANGLE:
        break;
    case READ_SINGLE_TURN_ANGLE:
        break;
    case READ_MOTOR_STATUS_1:
        _read_motor_status_1(arr);
        break;
    case CLEAR_ERROR_FLAG:
        break;
    case READ_MOTOR_STATUS_2:
        _read_motor_status_2(arr);
        break;
    case READ_MOTOR_STATUS_3:
        _read_motor_status_3(arr);
        break;
    case MOTOR_OFF:
        break;
    case MOTOR_STOP:
        break;
    case MOTOR_START:
        break;
    case TORQUE_CLOSED_LOOP:
        break;
    case SPEED_CLOSED_LOOP:
        break;
    case POSITION_CLOSED_LOOP_1:
        break;
    case POSITION_CLOSED_LOOP_2:
        break;
    case POSITION_CLOSED_LOOP_3:
        break;
    case POSITION_CLOSED_LOOP_4:
        break;
    default:
        Serial.println("Unrecognized");
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
    _motor_speed = _combine_two_bytes_int_16t(arr[5], arr[4]);
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

void RMD::_read_pid(uint8_t *arr)
{
    _current_loop_kp = (float)arr[2];
    _current_loop_ki = (float)arr[3];
    _speed_loop_kp = (float)arr[4];
    _speed_loop_ki = (float)arr[5];
    _position_loop_kp = (float)arr[6];
    _position_loop_ki = (float)arr[7];
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
