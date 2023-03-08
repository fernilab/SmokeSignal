#include <ArduinoBLE.h>

/*
 * NOTE on built in Seeeduino LEDs:
 * You first have to understand that the behavior of this LED is not as usual when controlled by the code.
 * The LED turns ON when we give a LOW signal and it turns OFF when we give a HIGH signal. 
 * This is because this LED is controlled by a common anode and will light up only with a low-level signal.
 */

/*
 * Needs a modified ArduinoCore-mbed. Look at https://github.com/Seeed-Studio/ArduinoCore-mbed/issues/13
 * There is an issue with using LEDG and reading PIN_VBAT in 2.9.1
 */

boolean debug = false;

// This UUID must match TX and RX
BLEService ledService("12b665c3-6546-4d19-8a87-cb2caa590510");
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("12b665c4-6546-4d19-8a87-cb2caa590510", BLERead | BLEWrite);

// How often should we read battery levels? Right now it is set to 10 minutes.
const unsigned long checkBatteryDelay = 5000;
const unsigned long blinkDelay        = 2500;
unsigned long nowMillis         = millis();
unsigned long startMillisBatt   = millis();
unsigned long startMillisBlinkR = millis();
const int battLow = 410;
int lowBattery = 0;

// Variables for buttons
const int ledOut = D0;

// Set LEDs
const int ledR = LEDR;  // pin to use for RED LED
const int ledG = LEDG;  // pin to use for GREEN LED
const int ledB = LEDB;  // pin to use for BLUE LED

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial);
  }
 
  // Set LED pins to output mode
  // Red is on, Green and Blue are off
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledOut, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, HIGH);
  digitalWrite(ledB, HIGH);
  digitalWrite(ledOut, LOW);

  // Enable battery monitoring
  pinMode(PIN_VBAT, INPUT);            //Battery Voltage monitoring pin
  pinMode(PIN_VBAT_ENABLE, OUTPUT);    //Enable Battery Voltage monitoring pin
  digitalWrite(PIN_VBAT_ENABLE, LOW);  //Enable

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
 
  // set advertised local name and service UUID:
  BLE.setLocalName("SmokeSignal Peripheral");
  BLE.setAdvertisedService(ledService);
 
  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
 
  // add service
  BLE.addService(ledService);
 
  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // set an initial value for the characteristic
  switchCharacteristic.setValue(0);
 
  // start advertising
  BLE.advertise();
 
  // print address
  Serial.print("Address: ");
  Serial.println(BLE.address());
  Serial.println("SmokeSignal nRF52840 Peripheral");
}

void loop() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();
  nowMillis = millis();
  if ((nowMillis - startMillisBatt) >= checkBatteryDelay) {
    checkVoltage();
  }
  blinkRed();
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected to central: ");
  Serial.println(central.address());
  digitalWrite(ledR, HIGH);
  digitalWrite(ledG, LOW);
  delay(500);
  digitalWrite(ledG, HIGH);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  digitalWrite(ledR, LOW);
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event: ");
  if (switchCharacteristic.value()) {  // any value other than 0
    Serial.println(F("LED On"));
    digitalWrite(ledB, LOW);           // will turn the LED on
    digitalWrite(ledOut, HIGH);        // turn on external LED
  } else {                             // a 0 value
    Serial.println(F("LED Off"));
    digitalWrite(ledB, HIGH);          // will turn the LED off
    digitalWrite(ledOut, LOW);         // turn off external LED
  }
}

void checkVoltage() {
  float voltage = analogRead(PIN_VBAT);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  startMillisBatt = millis();
  if (voltage < battLow) {
    lowBattery = 1;  // Set the global variable to blink red when battery is low.
  } else {
    lowBattery = 0;
  }
}

void blinkRed() {
  int ledRstatus;
  if (lowBattery && (nowMillis - startMillisBlinkR) >= blinkDelay) {
    if (digitalRead(ledR)) {
      digitalWrite(ledR, LOW);  // will turn the LED on
      digitalWrite(ledG, LOW);
    } else {
      digitalWrite(ledR, HIGH);  // will turn the LED off
      digitalWrite(ledG, HIGH);
    }
    startMillisBlinkR = millis();
  }
}
