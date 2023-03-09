#include "BattHelp.h"

// How often should we read battery levels? Right now it is set to 10 minutes.
const unsigned long checkBatteryDelay = 5000;
const unsigned long blinkDelay        = 2500;

// Battery level variables
const int battLow = 410;
int       lowBattery = 0;

// Timers
unsigned long startMillisBatt = 0;
unsigned long startMillisLED = 0;

// LED state
boolean state = false;


BatteryCheck::BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin)
{
  // Enable battery monitoring
  pinMode(vbatPin, INPUT);
  pinMode(vbatEnablePin, OUTPUT);    // Enable Battery Voltage monitoring pin
  digitalWrite(vbatEnablePin, LOW);  // Bring low to read battery levels
}

void BatteryCheck::checkVoltage(unsigned vbatPin) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillisBatt > checkBatteryDelay) {
    float voltage = analogRead(vbatPin);
    Serial.print("current: ");
    Serial.println(currentMillis);
    Serial.print("start: ");
    Serial.println(startMillisBatt);
    Serial.print("delay: ");
    Serial.println(checkBatteryDelay);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    startMillisBatt = millis();
    if (voltage < battLow) {
      lowBattery = 1;  // Set the global variable to blink red when battery is low.
    } else {
      lowBattery = 0;
    }
  }
}

void BatteryCheck::blinkLed(unsigned ledR, unsigned ledG, unsigned ledB) {
  unsigned long currentMillis = millis();

  if (lowBattery && currentMillis - startMillisLED > blinkDelay) {
    startMillisLED = millis();
    state = !state;
    blinkYellow(ledR, ledG, state);
  }
}

void blinkYellow(unsigned ledR, unsigned ledG, boolean state) {
  if (ledR) digitalWrite(ledR, state);
  if (ledG) digitalWrite(ledG, state);  
}
