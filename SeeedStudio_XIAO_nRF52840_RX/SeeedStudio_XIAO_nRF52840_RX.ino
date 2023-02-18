#include <ArduinoBLE.h>

boolean debug = 0;

// This UUID must match TX and RX
BLEService ledService("12b665c3-6546-4d19-8a87-cb2caa590510");
// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("12b665c3-6546-4d19-8a87-cb2caa590510", BLERead | BLEWrite);
 
// Set LED
const int ledR = LEDR; // pin to use for RED LED
const int ledG = LEDG; // pin to use for GREEN LED
const int ledB = LEDB; // pin to use for BLUE LED

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial);
  }
 
  // set LED pins to output mode
  // Red is on, Green and Blue are off
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, HIGH);
  digitalWrite(ledB, HIGH);
 
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
 
    while (1);
  }
 
  // set advertised local name and service UUID:
  BLE.setLocalName("XIAO");
  BLE.setAdvertisedService(ledService);
 
  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
 
  // add service
  BLE.addService(ledService);
 
  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
 
  // start advertising
  BLE.advertise();
 
  // print address
  Serial.print("Address: ");
  Serial.println(BLE.address());
 
  Serial.println("XIAO nRF52840 Peripheral");
}
 
void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();
  // RED LED is on unless we are connected
  digitalWrite(ledR, LOW);
 
  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // We are now connected so turn off RED LED and blink GREEN for 3 seconds
    digitalWrite(ledR, HIGH);
    digitalWrite(ledG, LOW);
    delay(3000);
    digitalWrite(ledG, HIGH);
 
    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {
        if (switchCharacteristic.value()) {  // any value other than 0
          Serial.println("LED On");
          digitalWrite(ledB, LOW);          // will turn the LED on
        } else {                             // a 0 value
          Serial.println(F("LED Off"));
          digitalWrite(ledB, HIGH);           // will turn the LED off
        }
      }
    }
 
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    // We got disconnected so turn on red again
    digitalWrite(ledR, LOW);
  }
}
