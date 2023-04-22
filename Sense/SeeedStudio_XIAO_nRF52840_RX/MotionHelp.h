#ifndef MotionHelp_h
#define MotionHelp_h
#include "LSM6DS3.h"
#include "Arduino.h"

class MotionHelp {
  public:
    MotionHelp();
    void IMUstart();
    void checkAcc();
    void goToPowerOff();
    void setupTiltInterrupt();
  private:
    LSM6DS3 myIMU;
    const unsigned long sleepyTime = 3600000; /* Sleep after 1 hour (3600000 ms) */
};

void int1ISR();

void sleepBlink();
#endif
