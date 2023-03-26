#include <ArduinoBLE.h>
#include <Wire.h>
#include "BattHelp.h"
#include "MotionHelp.h"


/* NOTE on built in Seeeduino LEDs:
   You first have to understand that the behavior of this LED is not as usual when controlled by the code.
   The LED turns ON when we give a LOW signal and it turns OFF when we give a HIGH signal. 
   This is because this LED is controlled by a common anode and will light up only with a low-level signal. */

/* Needs a modified ArduinoCore-mbed. Look at https://github.com/Seeed-Studio/ArduinoCore-mbed/issues/13
   There is an issue with using LEDG and reading PIN_VBAT in 2.9.1 */

boolean debug = false;
String peerMAC = "xx:xx:xx:xx:xx:xx";

BatteryCheck batt(PIN_VBAT, PIN_VBAT_ENABLE);
LEDs         leds(LEDR, LEDG, LEDB);
MotionHelp   mymotion;

BLEDevice peripheral;

/* This will be used to check double-clicks */
unsigned long switchOld   = millis();
unsigned long switchNow   = millis();

/* Setup button */
const int buttonPin = D0;
int oldButtonState  = LOW;

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial);
  }
  /* Configure the button pin as input */
  pinMode(buttonPin, INPUT_PULLUP);

  /* Initialize the Bluetooth® Low Energy hardware */
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

  /* Set the discovered event handle */
  BLE.setEventHandler(BLEDiscovered, bleCentralDiscoverHandler);

  /* Start scanning for peripherals */
  BLE.scanForName("SmokeSignal Peripheral");

  /* Print address */
  Serial.print("Address: ");
  Serial.println(BLE.address());
  Serial.println("SmokeSignal nRF52840 Central");

  /* Start accelerometer */
  mymotion.IMUstart();
}

void loop() {
  /* Poll for Bluetooth® Low Energy events */
  BLE.poll();
  /* We need to check this regardless of connected status
     Add the same thing to peripheral.connected loop */
  batt.checkVoltage(PIN_VBAT);
  leds.battStatus();
  leds.connectionStatus();
  mymotion.checkAcc();
}

void bleCentralDiscoverHandler(BLEDevice peripheral) {
  /* Discovered a peripheral */
  Serial.println("Discovered a peripheral");
  Serial.println("-----------------------");

  /* Print address */
  Serial.print("Address: ");
  Serial.println(peripheral.address());

  /* Print the local name, if present */
  if (peripheral.hasLocalName()) {
    Serial.print("Local Name: ");
    Serial.println(peripheral.localName());
  }

  /* Print the advertised service UUIDs, if present */
  if (peripheral.hasAdvertisedServiceUuid()) {
    Serial.print("Service UUIDs: ");
    for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
      Serial.print(peripheral.advertisedServiceUuid(i));
      Serial.print(" ");
    }
    Serial.println();
  }

  /* Print the RSSI */
  Serial.print("RSSI: ");
  Serial.println(peripheral.rssi());
  
  if (!peerMAC.equalsIgnoreCase(peripheral.address())) {
    Serial.print("Not an allowed peer: ");
    Serial.println(peerMAC + " =! " + peripheral.address());
    return;
  }

  if (peripheral.localName() != "SmokeSignal Peripheral") {
    Serial.print("Not SmokeSignal...");
    return;
  }

  /* Stop scanning */
  BLE.stopScan();

  system_control(peripheral);

  /* Peripheral disconnected, start scanning again */
  BLE.scanForName("SmokeSignal Peripheral");
}

void system_control(BLEDevice peripheral) {
  /* Connect to the peripheral */
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
    leds.connectionUpdate(true);
  } else {
    Serial.println("Failed to connect!");
    leds.connectionUpdate(false);
    return;
  }

  /* Discover peripheral attributes */
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  /* Retrieve the LED characteristic */
  BLECharacteristic ledCharacteristic = peripheral.characteristic("12b665c4-6546-4d19-8a87-cb2caa590510");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    /* While the peripheral is connected read the button pin */
    int buttonState = digitalRead(buttonPin);
    const unsigned long clickDebounce = 500;

    if (oldButtonState != buttonState) {
      /* Button changed */
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("button released");
        /* Write 0x00 to turn the LED off */
        ledCharacteristic.writeValue((byte)0x00);
        leds.ledChange(LEDB, true);
      } else {
        Serial.println("button pressed");
        /* Write 0x01 to turn the LED on */
        ledCharacteristic.writeValue((byte)0x01);
        leds.ledChange(LEDB, false);
        /* If there is a double-click enter the loop */
        if (checkClick()) {
          delay(clickDebounce);  /* We use this for button debounce */
          while(true) {
            if(!digitalRead(buttonPin)) {
              Serial.println("New button press.");
              break;
            }
          }
        }
      }
    }
    /* Run admin stuff at the end of button processing */
    batt.checkVoltage(PIN_VBAT);
    leds.battStatus();
    leds.connectionStatus();
    mymotion.checkAcc();
  }

  Serial.println("Peripheral disconnected");
  /* We got disconnected so turn on red again */
  leds.connectionUpdate(false);
}

int checkClick() {
  const unsigned long clickPeriod = 400;  // Any two clicks under 400 microseconds is a double-click
  switchNow = millis();
  if (switchOld && (switchNow - switchOld) <= clickPeriod) {
    switchOld = switchNow;
    Serial.println("Double click");
    return 1;
  } else {
    switchOld = switchNow;
    Serial.println("Single click");
    return 0;
  }
}
