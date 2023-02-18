#include <ArduinoBLE.h>
#include <U8x8lib.h>
#include <Wire.h>

/*
 * NOTE on built in Seeeduino LEDs:
 * You first have to understand that the behavior of this LED is not as usual when controlled by the code.
 * The LED turns ON when we give a LOW signal and it turns OFF when we give a HIGH signal. 
 * This is because this LED is controlled by a common anode and will light up only with a low-level signal.
 */

boolean debug = 0;

BLEDevice peripheral;

// variables for button
const int buttonPin = D1;
int oldButtonState = LOW;

// Set LED
const int ledR = LEDR; // pin to use for RED LED
const int ledG = LEDG; // pin to use for GREEN LED
const int ledB = LEDB; // pin to use for BLUE LED

void setup() {
  Serial.begin(9600);
  if (debug) {
    while (!Serial);
  }
  // configure the button pin as input
  pinMode(buttonPin, INPUT_PULLUP);

  // set LED pins to output mode
  // Red is on, Green and Blue are off
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledG, HIGH);
  digitalWrite(ledB, HIGH);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

  // start scanning for peripherals
  BLE.scanForName("XIAO");
}

void loop() {
  // check if a peripheral has been discovered
  peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    if (peripheral.localName() != "XIAO") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    system_control(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForName("XIAO");
  }
  delay(500);
}

void system_control(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
    // We are now connected so turn off RED LED and blink GREEN for 3 seconds
    digitalWrite(ledR, HIGH);
    digitalWrite(ledG, LOW);
    delay(3000);
    digitalWrite(ledG, HIGH);
  } else {
    Serial.println("Failed to connect!");
    // We got disconnected so turn on red again
    digitalWrite(ledR, LOW);
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("12b665c3-6546-4d19-8a87-cb2caa590510");

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
    // while the peripheral is connected
    // read the button pin
    int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("button released");
        // write 0x01 to turn the LED on
        ledCharacteristic.writeValue((byte)0x00);
        digitalWrite(ledB, HIGH);
      } else {
        Serial.println("button pressed");
        // write 0x00 to turn the LED off
        ledCharacteristic.writeValue((byte)0x01);
        digitalWrite(ledB, LOW);
      }
    }
  }

  Serial.println("Peripheral disconnected");
  // We got disconnected so turn on red again
  digitalWrite(ledR, LOW);
}
