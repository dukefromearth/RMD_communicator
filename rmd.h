#include "rmd_definitions.h"
#include <Arduino.h>

class RMD
{
public:
    RMD(int8_t frame_size = 8, uint16_t motor_id = 0x141);
    ~RMD(){};

    void print_data(char);

    void parse_reply(uint8_t *arr, int8_t len);

    void set_buf_for_read_cmd(uint8_t *buffer, uint8_t len, uint8_t cmd);

    int16_t get_frame_count()
    {
        return _frame_count;
    }

    uint16_t getID(void) { return _motor_id; }

private:
    int8_t _motor_temperature = 0; // unit 1 degree celcius/LSB
    float _voltage = 0.0;          // unit 0.1V/LSB
    uint8_t _error_state = 0;      // each bit represents a different motor state
    int8_t _frame_size = 8;
    float _motor_torque_current_amps = 0.0;
    int16_t _motor_speed = 0;
    uint16_t _encoder_position = 0;
    uint16_t _motor_id;
    float _phase_a_current = 0.0;
    float _phase_b_current = 0.0;
    float _phase_c_current = 0.0;
    int16_t _frame_count = 0;
    float _current_loop_kp = 0.0;
    float _current_loop_ki = 0.0;
    float _speed_loop_kp = 0.0;
    float _speed_loop_ki = 0.0;
    float _position_loop_kp = 0.0;
    float _position_loop_ki = 0.0;

    bool _does_equal_frame_size(int8_t len);

    void _read_motor_status_1(uint8_t *arr);

    void _read_motor_status_2(uint8_t *arr);

    void _read_motor_status_3(uint8_t *arr);

    void _read_pid(uint8_t *arr);

    uint16_t _combine_two_bytes_uint_16t(uint8_t low, uint8_t high);

    int16_t _combine_two_bytes_int_16t(uint8_t low, uint8_t high);
};