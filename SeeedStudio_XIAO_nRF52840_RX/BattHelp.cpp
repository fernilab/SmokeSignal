#include "BattHelp.h"

/* How often should we read battery levels and what should the blink delay be?                */
const unsigned long checkBatteryDelay = 60000;  /* Check every 60 seconds                     */
const unsigned long blinkDelay        = 20000;  /* When battery is low blink every 20 seconds */

/* Battery level variables */
int                 blinkCount     = 0;         /* This will keep track of blinks */
unsigned long       blinkStartTime = 0;
boolean             lowBattery     = false;     /* When this is set, start blinking  */
const unsigned long strobeDelay    = 125;       /* Delay in millis for strobe effect */

/* Connectivity status */
boolean connectionStatusFlag = false;


BatteryCheck::BatteryCheck(unsigned vbatPin, unsigned vbatEnablePin) {
  /* Enable battery monitoring */
  pinMode(vbatPin, INPUT);
  pinMode(vbatEnablePin, OUTPUT);    /* Enable Battery Voltage monitoring pin */
  digitalWrite(vbatEnablePin, LOW);  /* Bring low to read battery levels      */
  digitalWrite(P0_13, LOW);          /* High charging current                 */
}

void BatteryCheck::checkVoltage(unsigned vbatPin) {
  unsigned long currentTime = millis();
  if (currentTime - startTimeBatt > checkBatteryDelay) {
    float voltage = analogRead(vbatPin);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    startTimeBatt = currentTime;
    /* Once we hit a low battery event we set lowBattery to true
       and don't come out of it. Battery level fluctuates a bit
       so it would come in and out of lowBattery mode. */
    if (voltage < battLow) {
      lowBattery = true;
    }
    if (voltage < battCritical) {
      /* At this point shutdown. Battery is too low */
      NRF_POWER->SYSTEMOFF = 1;
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
  unsigned long currentTime = millis();

  if (lowBattery && currentTime - startTimeLED >= blinkDelay) {
    startTimeLED = currentTime;
    blinkCount = 0;
  }

  if (blinkCount < 2) {
    if (!state && (currentTime - blinkStartTime >= strobeDelay)) {
      /* Low battery is threat level yellow (R+G)*/
      LEDs::ledChange(LEDR, state);
      LEDs::ledChange(LEDG, state);
      state = true;
      blinkStartTime = currentTime;
    }
    else if (state && (currentTime - blinkStartTime >= strobeDelay)) {
      LEDs::ledChange(LEDR, state);
      LEDs::ledChange(LEDG, state);
      state = false;
      blinkCount++;
      blinkStartTime = currentTime;
    }
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
