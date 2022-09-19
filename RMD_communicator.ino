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
unsigned char buf[8];

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
  readPID();
  readReplyBlocking('\n');
  delay(1000);
}

void loop()
{
  readMotorStatus1();
  readReplyBlocking('\n');
  readMotorStatus2();
  readReplyBlocking('\n');
  readMotorStatus3();
  readReplyBlocking('\n');
  readPID();
  readReplyBlocking('\n');
  delay(5000);
  // delay(1000);
  // motor1.print_data();
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

void readReplyBlocking(char delim)
{
  while (CAN_MSGAVAIL != CAN.checkReceive()) // check if data coming
  {
  }
  while (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
    motor1.parse_reply(buf, len);
    motor1.print_data(delim);
  }
}