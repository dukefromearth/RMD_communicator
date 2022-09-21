// demo: CAN-BUS Shield, send data
// loovee@seeed.cc
#include "mcp_can.h"
#include <SPI.h>
#include "rmd.h"

#define StepValue 140
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

RMD motor1;

// variable to receive the message
unsigned char len = 0;
// variable to send the message
unsigned char buf[8] = {0};

void setup()
{
  Serial.begin(115200);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) // init can bus : baudrate = 500k
  {
    //        Serial.println("CAN BUS Shield init fail");
    //        Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  motorStart();
  readReplyBlocking();
  // delay(3000);
  // readEncoderOffset();
  // readReplyBlocking();
}

void loop()
{
  if (Serial.available() > 0)
  {
    int input = Serial.parseInt();
    setPosition4((float)input);
  }
  readAngle();
  readReplyBlocking();
  // readMotorStatus2();
  // readReplyBlocking();
  motor1.print_data(',');
}

void setPosition4(float input)
{
  motor1.set_multiturn_position(buf, input, 1000);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readEncoderOffset()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_MULTITURN_ENCODER_POSITION_OFFSET);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readAngle()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_MULTITURN_ENCODER_POSITION);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void motorStart()
{
  motor1.set_buf_for_read_cmd(buf, 8, MOTOR_START);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readMotorStatus1()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_MOTOR_STATUS_1);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readMotorStatus2()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_MOTOR_STATUS_2);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readMotorStatus3()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_MOTOR_STATUS_3);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readPID()
{
  motor1.set_buf_for_read_cmd(buf, 8, READ_PID);
  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void setPID()
{
  // motor1.set_buf_for_read_cmd(buf, 8, READ_PID);
  buf[0] = '0x31';
  buf[1] = '0x00';
  buf[2] = '0x00';
  buf[3] = '0x00';
  buf[4] = '0x00';
  buf[5] = '0x00';
  buf[6] = '0x00';
  buf[7] = '0x00';

  CAN.sendMsgBuf(motor1.getID(), 0, 8, buf);
}

void readReplyBlocking()
{
  while (CAN_MSGAVAIL != CAN.checkReceive()) // check if data coming
  {
  }
  while (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
    motor1.parse_reply(buf, len);
    // printReply(buf, 8);
  }
}

void printReply(uint8_t *buffer, uint8_t len)
{
  for (int i = 0; i < len; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print('\t');
  }
  Serial.println();
}