#include <ArduinoBLE.h>
#include "BattHelp.h"
#include "MotionHelp.h"

/* NOTE on built in Seeeduino LEDs:
   You first have to understand that the behavior of this LED is not as usual when controlled by the code.
   The LED turns ON when we give a LOW signal and it turns OFF when we give a HIGH signal.
   This is because this LED is controlled by a common anode and will light up only with a low-level signal. */

/* Needs a modified ArduinoCore-mbed. Look at https://github.com/Seeed-Studio/ArduinoCore-mbed/issues/13
   There is an issue with using LEDG and reading PIN_VBAT in 2.9.1 */

boolean debug = false;
String peerMAC = "xx:xx:xx:xx:xx:xx";  /* Set peer MAC address for simple allowlist */

BatteryCheck batts(PIN_VBAT, PIN_VBAT_ENABLE);
LEDs         leds(LEDR, LEDG, LEDB);
MotionHelp   mymotion;

/* This UUID must match TX and RX */
BLEService ledService("12b665c3-6546-4d19-8a87-cb2caa590510");
/* Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central */
BLEByteCharacteristic switchCharacteristic("12b665c4-6546-4d19-8a87-cb2caa590510", BLERead | BLEWrite);

/* Setup output LED */
const int ledOut = D0;

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial);
  }

  /* Setup output LED */
  pinMode(ledOut, OUTPUT);
  leds.ledChange(ledOut, false);

  /* Begin BLE initialization */
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  /* Set advertised local name and service UUID: */
  BLE.setLocalName("SmokeSignal Peripheral");
  BLE.setAdvertisedService(ledService);

  /* Add the characteristic to the service */
  ledService.addCharacteristic(switchCharacteristic);

  /* Add service */
  BLE.addService(ledService);

  /* Assign event handlers for connected, disconnected to peripheral */
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  /* Assign event handlers for characteristic */
  switchCharacteristic.setEventHandler(BLEWritten, switchCharacteristicWritten);
  /* Set an initial value for the characteristic */
  switchCharacteristic.setValue(0);

  /* Start advertising */
  BLE.advertise();

  /* Print address */
  Serial.print("Address: ");
  Serial.println(BLE.address());
  Serial.println("SmokeSignal nRF52840 Peripheral");

  /* Start accelerometer */
  mymotion.IMUstart();
}

void loop() {
  /* Poll for Bluetooth® Low Energy events */
  BLE.poll();
  batts.checkVoltage(PIN_VBAT);
  leds.battStatus();
  leds.connectionStatus();
  mymotion.checkAcc();
}

void blePeripheralConnectHandler(BLEDevice central) {
  /* if peerMAC does not match, disconnect */
  if (!peerMAC.equalsIgnoreCase(central.address())) {
    Serial.print("Not an allowed peer: ");
    Serial.println(peerMAC + " =! " + central.address());
    central.disconnect();
    return;
  }
  /* Central connected event handler */
  Serial.print("Connected to central: ");
  Serial.println(central.address());
  leds.connectionUpdate(true);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  /* Central disconnected event handler */
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
  leds.connectionUpdate(false);
}

void switchCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  /* Central wrote new value to characteristic, update LED */
  Serial.print("Characteristic event: ");
  if (switchCharacteristic.value()) {
    Serial.println(F("LED On"));
    leds.ledChange(LEDB, false);            /* Will turn the status LED on */
    leds.ledChange(ledOut, true);           /* Turn on external LED */
  } else {
    Serial.println(F("LED Off"));
    leds.ledChange(LEDB, true);             /* Will turn the status LED off */
    leds.ledChange(ledOut, false);          /* Turn off external LED */
  }
}
