#include <stdint.h>
#include <Arduino.h>

class waveshareMotor {
public:
  // CONSTRUCTORS
  waveshareMotor();
  void setM1Speed(int speed);
  int getM1Speed();
  void setM2Speed(int speed);
  int getM2Speed();
  void setM3Speed(int speed);
  int getM3Speed();
  void setM4Speed(int speed);
  int getM4Speed();


private:
  byte CRC8(const byte *data, byte len);
  uint8_t checkSpeed[10] = { 0x01, 0x74, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t Speed[10] = { 0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  bool lock1 = false;
  bool lock2 = false;
  bool lock3 = false;
  bool lock4 = false;
};
