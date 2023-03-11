#include "BattHelp.h"

/* How often should we read battery levels and what should the blink delay be? */
const unsigned long checkBatteryDelay = 15000;
const unsigned long blinkDelay        = 1000;

/* Battery level variables */
boolean   lowBattery = false;

/* Connectivity status */
boolean connectionStatusFlag = false;


BatteryCheck::BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin) {
  /* Enable battery monitoring */
  pinMode(vbatPin, INPUT);
  pinMode(vbatEnablePin, OUTPUT);    /* Enable Battery Voltage monitoring pin */
  digitalWrite(vbatEnablePin, LOW);  /* Bring low to read battery levels */
}

void BatteryCheck::checkVoltage(unsigned vbatPin) {
  unsigned long currentMillis = millis();
  if (currentMillis - startMillisBatt > checkBatteryDelay) {
    float voltage = analogRead(vbatPin);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    startMillisBatt = millis();
    if (voltage < battLow) {
      lowBattery = true;
    } else {
      lowBattery = false;
    }
  }
}

LEDs::LEDs(unsigned ledR, unsigned ledG, unsigned ledB) {
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, HIGH);
  digitalWrite(ledB, HIGH);
}

void LEDs::ledChange(unsigned ledPin, boolean ledState) {
  digitalWrite(ledPin, ledState);
}

void LEDs::battStatus() {
  unsigned long currentMillis = millis();

  if (lowBattery && currentMillis - startMillisLED > blinkDelay) {
    startMillisLED = millis();
    state = !state;
    /* Low battery is threat level yellow */
    LEDs::ledChange(LEDR, state);
    LEDs::ledChange(LEDG, state);
  }
}

void LEDs::connectionUpdate(boolean connectionState) {
  connectionStatusFlag = connectionState;
  if (connectionStatusFlag) {
    /* Flash green only briefly to show connected */
    LEDs::ledChange(LEDR, true);
    LEDs::ledChange(LEDB, true);
    LEDs::ledChange(LEDG, false);
    delay(500);
    LEDs::ledChange(LEDG, true);
  }
}

void LEDs::connectionStatus() {
  /* Loss of connection is threat level red */
  if (!connectionStatusFlag) LEDs::ledChange(LEDR, false);
}
