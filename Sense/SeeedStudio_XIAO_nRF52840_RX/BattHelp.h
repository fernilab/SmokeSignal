#ifndef BattHelp_h
#define BattHelp_h
#include "Arduino.h"

class BatteryCheck {
  public:
  BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin);
  void checkVoltage(unsigned vbatPin);

  private:
  unsigned long startTimeBatt = 0;
  const int     battLow       = 340;  /* 340 Start battery warnings. */
  const int     battCritical  = 333;  /* 333 Battery depleated. Shutdown. */
};

class LEDs {
  public:
  LEDs(unsigned ledR, unsigned ledG, unsigned ledB);
  void ledChange(unsigned ledPin, boolean ledState);
  void battStatus();
  void connectionUpdate(boolean connectionState);
  void connectionStatus();

  private:
  unsigned long startTimeLED = 0;
  boolean       state        = false;
};

#endif
