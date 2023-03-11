#ifndef BattHelp_h
#define BattHelp_h
#include "Arduino.h"


class BatteryCheck {
  public:
  BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin);
  void checkVoltage(unsigned vbatPin);

  private:
  unsigned long startMillisBatt = 0;
  const int     battLow = 355;
};

class LEDs {
  public:
  LEDs(unsigned ledR, unsigned ledG, unsigned ledB);
  void ledChange(unsigned ledPin, boolean ledState);
  void battStatus();
  void connectionUpdate(boolean connectionState);
  void connectionStatus();

  private:
  unsigned long startMillisLED = 0;
  boolean state = false;
};

#endif
