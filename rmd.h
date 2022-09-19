#include "rmd_definitions.h"

class RMD{
    public: 
    RMD(){};
    ~RMD(){};
    void parse_reply(uint8_t *arr, int8_t len){
        if(!_check_frame_size(len))
            return;
        _parse_reply(arr);
    }

    void print_data(){
        _print_data();
    }

    private:
        int8_t _motor_temperature = 0;       // unit 1 degree celcius/LSB
        float _voltage = 0;               // unit 0.1V/LSB
        uint8_t _error_state = 0;            // each bit represents a different motor state
        int8_t _frame_size = 8;
        float _motor_torque_current_amps = 0;
        int16_t _motor_speed = 0;
        uint16_t _encoder_position = 0;

        bool _check_frame_size(int8_t len){
            return (len == _frame_size);
        }

        void _print_data(){
            // Serial.print("Temperature: ");
            Serial.print(_motor_temperature);
            Serial.print('\t');
            // Serial.print("Voltage: ");
            Serial.print(_voltage);
            Serial.print('\t');
            // Serial.print("Current: ");
            Serial.print(_motor_torque_current_amps);
            Serial.print('\t');
            // Serial.print("Speed: ");
            Serial.print(_motor_speed);
            Serial.print('\t');
            // Serial.print("Encoder: ");
            Serial.print(_encoder_position);
            Serial.print('\t');
            // Serial.print("Error: ");
            Serial.println(_error_state);
        }

        void _parse_reply(uint8_t *arr)
        {
            switch (arr[0])
            {
            case READ_PID:
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
                Serial.println("Default");
                break;
        }
    }

    /// @brief  Reads in motor temperature, voltage, and error state
    ///         
    /// @param arr 8 length array of type uint8_t
    void _read_motor_status_1(uint8_t *arr){
        _voltage = ((float)(_combine_two_bytes_uint_16t(arr[5], arr[4])))*0.1;
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
    void _read_motor_status_2(uint8_t *arr){
        _motor_temperature = (int8_t)arr[1];
        _motor_torque_current_amps = ((float)_combine_two_bytes_int_16t(arr[3], arr[2])) * 0.01;
        _motor_speed = _combine_two_bytes_int_16t(arr[5], arr[4]);
        _encoder_position = _combine_two_bytes_uint_16t(arr[7], arr[6]);
        return;
    }

    uint16_t _combine_two_bytes_uint_16t(uint8_t low, uint8_t high){
        return ((low << 8) | high);
    }

    int16_t _combine_two_bytes_int_16t(uint8_t low, uint8_t high){
        return ((low << 8) | high);
    }
};