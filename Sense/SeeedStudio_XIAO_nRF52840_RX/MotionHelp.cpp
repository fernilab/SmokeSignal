#include "MotionHelp.h"
#include "LSM6DS3.h"
#include "Wire.h"

/* Define constants for the maximum allowed deviation in acceleration values */
const float MAX_ACCEL_DEVIATION = 0.25;

/* Use this to check if device is stationary */
unsigned long lastMoveTime = 0;

/* Accelerometer Interrupt Pin */
#define int1Pin PIN_LSM6DS3TR_C_INT1

/* Amount of received interrupts, it serves no purpose at the moment */
uint8_t interruptCount = 0;

/* Define variables for tracking the acceleration values and the last time they were updated */
float lastAccelX, lastAccelY, lastAccelZ;

MotionHelp::MotionHelp() : myIMU(I2C_MODE, 0x6A) {}

void MotionHelp::IMUstart() {
  myIMU.settings.gyroEnabled = 0;  /* Gyro currently not used, disabled to save power */
  if (myIMU.begin() != 0) {
      Serial.println("IMU start error");
  } else {
      Serial.println("IMU start OK");
      MotionHelp::setupTiltInterrupt();
      pinMode(int1Pin, INPUT);
      attachInterrupt(digitalPinToInterrupt(int1Pin), int1ISR, RISING);
  }
}

void MotionHelp::checkAcc() {
  /* Read acceleration values */
  float accelX = myIMU.readFloatAccelX();
  float accelY = myIMU.readFloatAccelY();
  float accelZ = myIMU.readFloatAccelZ();

  /* Check if the acceleration values have remained constant */
  bool stationary = true;
  if (abs(accelX - lastAccelX) > MAX_ACCEL_DEVIATION ||
      abs(accelY - lastAccelY) > MAX_ACCEL_DEVIATION ||
      abs(accelZ - lastAccelZ) > MAX_ACCEL_DEVIATION) {
    stationary = false;
    lastMoveTime = millis();  /* We moved, so update lastMoveTime */
  }
  
  /* Update the last acceleration values */
  lastAccelX = accelX;
  lastAccelY = accelY;
  lastAccelZ = accelZ;

  /* If enough time has passed since the last movement we go to sleep */
  unsigned long elapsedTime = millis() - lastMoveTime;
  if (elapsedTime > sleepyTime) {
    MotionHelp::goToPowerOff();
  }
}

void MotionHelp::goToPowerOff() {
  Serial.println("Shutting down.");
  sleepBlink();
  Serial.println("Setting up wakeup call.");
  /* Ensure interrupt pin from IMU is set to wake up device */
  nrf_gpio_cfg_sense_input(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  /* Brief delay to make sure the changes registered */
  delay(1000);
  Serial.println("Going to system OFF");
  NRF_POWER->SYSTEMOFF = 1;
}

void MotionHelp::setupTiltInterrupt() {
  // Set accelerometer measurement range and bandwidth
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL,    0x11);  /* 12.5Hz, 2g, 400Hz                                 */
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1,    0xEE);  /* INT_EN, acc lo-power, gyro power-down, TAP XYZ EN */
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D,  0x61);  /* D4D_EN, 50 deg, bits * 0.03125g                   */
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2,    0x15);  /* DUR: 2.56s, QUIET: .32s, SHOCK: .64s              */
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x81);  /* SINGNLE DOUBLE EN, bits * .03125g                 */
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG,     0x6E);  /* SINGLE, WU, DOUBLE, 6D, and TILT                  */
}

void int1ISR()
{
  interruptCount++;  /* The goggles, they do nothing! */
}

void sleepBlink() {
  for (int blinkCounter = 0; blinkCounter <=3; blinkCounter++) {
    digitalWrite(LEDR, LOW);
    delay(125);
    digitalWrite(LEDR, HIGH);
    delay(125);
  }
  /* Turn off LEDs */
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}
