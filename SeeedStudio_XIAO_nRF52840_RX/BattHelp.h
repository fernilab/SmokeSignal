#ifndef BattHelp_h
#define BattHelp_h

#include "Arduino.h"

class BatteryCheck
{
  public:
  BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin);
  void checkVoltage(unsigned vbatPin, unsigned long delay_time);
  void blinkRed(unsigned ledPin, unsigned long delay_time);

  private:
  unsigned long startMillisBatt = 0;
  unsigned long startMillisLED = 0;
  boolean       state = false;
  int           lowBattery = 0;
  const int     battLow = 410;
};

#endif
