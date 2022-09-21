#define READ_PID 0x30
#define WRITE_PID_TO_RAM 0x31
#define WRITE_PID_TO_ROM 0x32
#define READ_ACCELERATION 0x33
#define WRITE_ACCELERATION_TO_RAM 0x34
#define READ_ENCODER 0x90
#define WRITE_ENCODER_OFFSET 0x91
#define WRITE_CURRENT_POS_TO_ROM_AS_MOTOR_ZERO 0x19
#define READ_MULTI_TURN_ANGLE 0x92
#define READ_SINGLE_TURN_ANGLE 0x94
#define READ_MOTOR_STATUS_1 0x9A
#define CLEAR_ERROR_FLAG 0x9B
#define READ_MOTOR_STATUS_2 0x9C
#define READ_MOTOR_STATUS_3 0x9D
#define MOTOR_OFF 0x80
#define MOTOR_STOP 0x81
#define MOTOR_START 0x88
#define TORQUE_CLOSED_LOOP 0xA1
#define SPEED_CLOSED_LOOP 0xA2
#define POSITION_CLOSED_LOOP_1 0xA3
#define SET_MULTITURN_POSITION 0xA4
#define POSITION_CLOSED_LOOP_3 0xA5
#define POSITION_CLOSED_LOOP_4 0xA6
#define READ_MULTITURN_ENCODER_POSITION 0x60
#define READ_MULTITURN_ORIGINAL_ENCODER_POSITION 0x61
#define READ_MULTITURN_ENCODER_POSITION_OFFSET 0x62

/*
Command name Command data
Read position loop KP Parameter 0x30
Read position loop KI Parameter 0x31
Read Velocity loop KP Parameter 0x32
Read Velocity loop KI Parameter 0x33
Read Torque loop KP Parameter 0x34
Read Torque loop KI Parameter 0x35
Write Position loop KP to RAM 0x36
Write Position loop KI to RAM 0x37
Write Velocity loop KP to RAM 0x38
Write Velocity loop KI to RAM 0x39
Write Torque loop KP to RAM 0x3A
Write Torque loop KI to RAM 0x3B
Write Position loop KP to ROM 0x3C
Write Position loop KI to ROM 0x3D
Write Velocity loop KP to ROM 0x3E
Write Velocity loop KI to ROM 0x3F
Write Torque loop KP to ROM 0x40
Write Torque loop KI to ROM 0x41
Read Acceleration 0x42
Write acceleration data to RAM 0x43
Read multiturn encoder position 0x60
Read multiturn encoder original position 0x61
Read multiturn encoder offset 0x62
Write multiturn encoder value to ROM as motor zero 0x63
Write multiturn encoder current position to ROM as motor zero 0x64
Read multiturn turns angle 0x92
Read motor status 1 and error flag 0x9A
Read motor status 2 0x9C
Read motor status 3 0x9D
Motor off 0x80
Motor stop 0x81
Motor enable 0x88
Torque closed-loop contol 0xA1
Velocity closed-loop contol 0xA2
Multi-Position closed-loop control 0xA4
Incremental Position closed-loop control 0xA8
Read motor curent working mode 0x70
第5 页共43 页
Read motor current power 0x71
Read backup power voltage 0x72
TF Command 0x73
System reset command 0x76
Brake unlock command （motor turn freely ） 0x77
Brake lock (motor shaft locked) 0x78
Setup and read CAN ID 0x79
Read System running time 0xB1
Read firmware version 0xB2
Setup Communication interruption protection time 0xB3

*/