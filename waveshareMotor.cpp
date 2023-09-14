#include "waveshareMotor.h"
#include <Arduino.h>

#define HWSERIAL_M1 Serial5
#define HWSERIAL_M2 Serial8
#define HWSERIAL_M3 Serial2
#define HWSERIAL_M4 Serial7


// Constructors ////////////////////////////////////////////////////////////////

waveshareMotor::waveshareMotor()  //constructor
{
}

void waveshareMotor::setM1Speed(int speed) {
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  speed = -1*speed;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  if (lock1 == false) {
    lock1 = true;
    HWSERIAL_M1.write(Speed, sizeof(Speed));
    int ok = 0;
    while (HWSERIAL_M1.available() < 10) {
      /*ok++;
      if (ok > 10000) {
        break;
      }*/
    }  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M1.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock1 = false;
  }
}

int waveshareMotor::getM1Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock1 == false) {
    lock1 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    HWSERIAL_M1.write(checkSpeed, sizeof(checkSpeed));
    while (HWSERIAL_M1.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M1.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock1 = false;
  }
  return number;
}

void waveshareMotor::setM2Speed(int speed) {
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  if (lock2 == false) {
    lock2 = true;
    HWSERIAL_M2.write(Speed, sizeof(Speed));
    while (HWSERIAL_M2.available() < 10) {}  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M2.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock2 = false;
  }
}

int waveshareMotor::getM2Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock2 == false) {
    lock2 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    HWSERIAL_M2.write(checkSpeed, sizeof(checkSpeed));
    while (HWSERIAL_M2.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M2.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock2 = false;
  }
  return number;
}

void waveshareMotor::setM3Speed(int speed) {
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  speed = -1*speed;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  if (lock3 == false) {
    lock3 = true;
    HWSERIAL_M3.write(Speed, sizeof(Speed));
    while (HWSERIAL_M3.available() < 10) {}  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M3.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock3 = false;
  }
}

int waveshareMotor::getM3Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock3 == false) {
    lock3 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    HWSERIAL_M3.write(checkSpeed, sizeof(checkSpeed));
    while (HWSERIAL_M3.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M3.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock3 = false;
  }
  return number;
}

void waveshareMotor::setM4Speed(int speed) {
  uint8_t buffer[10];
  int number;
  uint8_t byte0;
  uint8_t byte1;
  byte0 = (speed >> 8) & 0xFF;
  byte1 = (speed >> 0) & 0xFF;

  Speed[2] = byte0;
  Speed[3] = byte1;

  Speed[9] = CRC8(Speed, 9);
  if (lock4 == false) {
    lock4 = true;
    HWSERIAL_M4.write(Speed, sizeof(Speed));
    while (HWSERIAL_M4.available() < 10) {}  // Wait 'till there are 15 Bytes waiting
    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M4.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock4 = false;
  }
}

int waveshareMotor::getM4Speed() {
  int number = 0;
  uint8_t buffer[10];
  if (lock4 == false) {
    lock4 = true;
    checkSpeed[9] = CRC8(checkSpeed, 9);
    HWSERIAL_M4.write(checkSpeed, sizeof(checkSpeed));
    while (HWSERIAL_M4.available() < 10) {}

    for (int n = 0; n < 10; n++)
      buffer[n] = HWSERIAL_M4.read();
    number = (int16_t)(buffer[4] << 8) + buffer[5];

    lock4 = false;
  }
  return number;
}

byte waveshareMotor::CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
