#ifndef BattHelp_h
#define BattHelp_h
#include "Arduino.h"


class BatteryCheck
{
  public:
  BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin);
  void checkVoltage(unsigned vbatPin);
  void blinkLed(unsigned ledR, unsigned ledG, unsigned ledB);

  private:
  unsigned long startMillisBatt = 0;
  unsigned long startMillisLED = 0;
  boolean       state = false;
  int           lowBattery = 0;
  const int     battLow = 410;
};

void blinkYellow(unsigned ledR, unsigned ledG, boolean state);

#endif
