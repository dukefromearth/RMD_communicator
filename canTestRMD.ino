// demo: CAN-BUS Shield, send data
// loovee@seeed.cc
#include "mcp_can.h"
#include <SPI.h>
#include "ArrbotMonitor.h"
#include "rmd.h"


/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

// Define Joystick connection pins
#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

//Define LED pins
#define LED2 8
#define LED3 7

#define StepValue 140
const int SPI_CS_PIN = 9;

MCP_CAN CAN(SPI_CS_PIN);                               // Set CS pin

RMD motor1;

long GenPos = -19784;
long GenVel = 600;

// variable to receive the message
unsigned char len = 0;
// variable to send the message
unsigned char buf[8];

int16_t encoderPos = 0;
int16_t originalEncoderPos = 0;
int16_t encoderOffset = 0;


void setup()
{
  SERIAL.begin(115200);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 500k
  {
    //        SERIAL.println("CAN BUS Shield init fail");
    //        SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }

}

uint32_t globalSpeed = 0;
uint32_t speedInc = 5000;

long int globalTorque = 0;
int torqueInc = 5;

long int globalPos = 0;
int posInc = 2500;

void loop()
{

// //  readMotorStatus();
  if (Serial.available() > 0) {
    char input = Serial.parseInt();
    if (input == 1) {
      startMotor();
    } else if (input == 2) {
      stopMotor();
    } else if(input == 3){
      sendSpeed(globalSpeed += speedInc);
    }else if(input == 4){
      sendSpeed(globalSpeed -= speedInc);
    }else if(input == 5){
      sendTorque(globalTorque += torqueInc);
    }else if(input == 6){
      sendTorque(globalTorque -= torqueInc);
    }else if(input == 7){
      sendPos(globalPos += posInc);
    }else if(input == 8){
      sendPos(globalPos -= posInc);
    }
  }
    
// //   }
// //   delay(100);                       // send data per 100ms
//   readMotorStatus1();
//   delay(1);
//   readReply();
//   readMotorStatus2();
//   delay(1);
//   readReply();
//   motor1.print_data();
//   delay(10);

}

void readMotorStatus1() {
  buf[0] = 0x9A;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
}

void readReply(){
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
      unsigned long canId = CAN.getCanId();
      motor1.parse_reply(buf, len);
    }
}

void readMotorStatus2() {
  buf[0] = 0x9C;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
}

void readEncoder() {
  buf[0] = 0x90;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  delay(1);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
    //        for(int i = 0; i<len; i++)    // print the data
    //        {
    //            SERIAL.print(buf[i], HEX);
    //            SERIAL.print("\t");
    //        }
    encoderPos = combineLowHighByte(buf[2], buf[3]);
    originalEncoderPos = combineLowHighByte(buf[4], buf[5]);
    encoderOffset = combineLowHighByte(buf[6], buf[7]);
    //        DISPLAY(17000);
    MONITOR(encoderPos);
    MONITOR2("originalEncoderPos", originalEncoderPos);
    //        MONITOR3("encoderOffset", encoderOffset);
    MONITOR_ENDL();

    //        SERIAL.print(encoderPos);
    //        SERIAL.print(",");
    //        SERIAL.print(originalEncoderPos);
    //        SERIAL.print("\t");
    //        SERIAL.print(encoderOffset);
    //        SERIAL.println();

    //        SERIAL.println(buf[7]);
    //        unsigned int p_in = (buf[7] << 8) | buf[6];
    //        SERIAL.println(p_in);
  }
}

double combineLowHighByte(unsigned char low, unsigned char high) {
  return ((double)(high << 8)) + ((double)low);
}

long long combineLowHighByteLong(unsigned char low, unsigned char high) {
  return ((long long)(high << 8)) + ((long long)low);
}

void sendTorque(int targetTorque){


  union
  {
     int t;
     unsigned char b[2];
  }myTorque;

  myTorque.t = targetTorque;
  
  buf[0] = 0xA1;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = myTorque.b[0];
  buf[5] = myTorque.b[1];
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  delay(1);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
            for(int i = 0; i<len; i++)    // print the data
            {
                SERIAL.print(buf[i], HEX);
                SERIAL.print("\t");
            }
            Serial.println();
  }

}

void sendSpeed(uint32_t targetSpeed){


  union
  {
     uint32_t spd;
     unsigned char b[4];
  }mySpeed;

  mySpeed.spd = targetSpeed;
  
  buf[0] = 0xA2;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0xA0;
  buf[5] = 0x86;
  buf[6] = 0x01;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  delay(1);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
            for(int i = 0; i<len; i++)    // print the data
            {
                SERIAL.print(buf[i], HEX);
                SERIAL.print("\t");
            }
            Serial.println();
  }

}

void sendPos(int pos) {

  union
  {
     uint16_t s;
     unsigned char b[2];
  }mySpeed;

  mySpeed.s = 20000;
  Serial.println(mySpeed.b[0], HEX);
  Serial.println(mySpeed.b[1], HEX);
//  return;
  
  union
  {
     long int p;
     unsigned char b[4];
  }myPos;

  myPos.p = pos;
  
  buf[0] = 0xA4;
  buf[1] = 0x00;
  buf[2] = mySpeed.b[0];
  buf[3] = mySpeed.b[1];
  buf[4] = myPos.b[0];
  buf[5] = myPos.b[1];
  buf[6] = myPos.b[2];
  buf[7] = myPos.b[3];
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  delay(1);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
            for(int i = 0; i<len; i++)    // print the data
            {
                SERIAL.print(buf[i], HEX);
                SERIAL.print("\t");
            }
            Serial.println();
  }
}

void startMotor() {
  //Start motor
  buf[0] = 0x88;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  //receive message
  delay(3);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
    SERIAL.print((buf[6] << 8) | buf[7]);
    SERIAL.print("\t");
    unsigned int EstPos = (buf[7] << 8) | buf[6];
    if (EstPos > 18000) {
      GenPos = -36000 + EstPos;
    } else {
      GenPos = EstPos;
    }
    SERIAL.print(buf[6], HEX);
    SERIAL.print("\t");
    SERIAL.print(buf[7], HEX);
    SERIAL.print("\t");
    SERIAL.print(EstPos);
    SERIAL.print("\t");
    SERIAL.println(GenPos);
  }
  delay(1000);
  SERIAL.println("Motor Start");
}
//
//void readPos(){
//  //read initial position
//    //motor 1
//    buf[0] = 0x94;
//    buf[1] = 0x00;
//    buf[2] = 0x00;
//    buf[3] = 0x00;
//    buf[4] = 0x00;
//    buf[5] = 0x00;
//    buf[6] = 0x00;
//    buf[7] = 0x00;
//    CAN.sendMsgBuf(0x141, 0, 8, buf);
//    //receive message
//    delay(3);
//    if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
//    {
//        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
//        unsigned long canId = CAN.getCanId();
//        SERIAL.print((buf[6] << 8) | buf[7]);
//        SERIAL.print("\t");
//        unsigned int EstPos = (buf[7] << 8) | buf[6];
//        if (EstPos>18000) {
//          GenPos = -36000+EstPos;
//        } else {
//          GenPos = EstPos;
//        }
//        SERIAL.print(buf[6], HEX);
//        SERIAL.print("\t");
//        SERIAL.print(buf[7], HEX);
//        SERIAL.print("\t");
//        SERIAL.print(EstPos);
//        SERIAL.print("\t");
//        SERIAL.println(GenPos);
//    }
//    delay(3000);
//}

void stopMotor() {

  //read initial position
  //motor 1
  buf[0] = 0x81;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  //receive message
  delay(3);
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    unsigned long canId = CAN.getCanId();
    SERIAL.print((buf[6] << 8) | buf[7]);
    SERIAL.print("\t");
    unsigned int EstPos = (buf[7] << 8) | buf[6];
    if (EstPos > 18000) {
      GenPos = -36000 + EstPos;
    } else {
      GenPos = EstPos;
    }
    SERIAL.print(buf[6], HEX);
    SERIAL.print("\t");
    SERIAL.print(buf[7], HEX);
    SERIAL.print("\t");
    SERIAL.print(EstPos);
    SERIAL.print("\t");
    SERIAL.println(GenPos);
  }
  Serial.println("Motor Stopped");
  delay(3000);
}

// END FILE
