#include <Arduino.h>
#include <Wire.h>

void scanI2CBus() {
  byte foundCount = 0;

  Serial.println(F("Scanning I2C bus..."));

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("Found I2C device at 0x"));
      if (address < 16) {
        Serial.print(F("0"));
      }
      Serial.println(address, HEX);
      foundCount++;
    } else if (error == 4) {
      Serial.print(F("Unknown error at 0x"));
      if (address < 16) {
        Serial.print(F("0"));
      }
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) {
    Serial.println(F("No I2C devices found"));
  } else {
    Serial.print(F("Done. Devices found: "));
    Serial.println(foundCount);
  }

  Serial.println();
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
  }

  Wire.begin();
  Serial.println(F("I2C Scanner ready"));
  Serial.println(F("Arduino Uno/Nano I2C: SDA=A4, SCL=A5"));
  scanI2CBus();
}

void loop() {
  delay(3000);
  scanI2CBus();
}
