#include "BattHelp.h"

unsigned long startMillisBatt = 0;
unsigned long startMillisLED = 0;
boolean       state = false;
int           lowBattery = 0;
const int     battLow = 410;

BatteryCheck::BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin)
{
  // Enable battery monitoring
  pinMode(vbatPin, INPUT);
  pinMode(vbatEnablePin, OUTPUT);    // Enable Battery Voltage monitoring pin
  digitalWrite(vbatEnablePin, LOW);  // Bring low to read battery levels
}

void BatteryCheck::checkVoltage(unsigned vbatPin, unsigned long delay_time) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillisBatt > delay_time) {
    float voltage = analogRead(vbatPin);
    Serial.print("current: ");
    Serial.println(currentMillis);
    Serial.print("start: ");
    Serial.println(startMillisBatt);
    Serial.print("delay: ");
    Serial.println(delay_time);
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

void BatteryCheck::blinkRed(unsigned ledPin, unsigned long delay_time) {
  unsigned long currentMillis = millis();

  if (currentMillis - startMillisLED > delay_time) {
    state = !state;
    digitalWrite(ledPin, state);
    startMillisLED = millis();
  }
}
