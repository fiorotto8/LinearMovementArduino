// the setup routine runs once when you press reset:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


Adafruit_BME680 bme; // I2C
int incomingByte = 0; // for incoming serial data

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}
// the loop routine runs over and over again forever:
void loop() {

  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    //82 is the ascii code or 'R'
    if (incomingByte == 82){
      unsigned long endTime = bme.beginReading();
       if (endTime == 0) {
         Serial.println(F("Failed to begin reading :("));
         return;
       }
      if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        return;
      }
/*
      Serial.print(incomingByte);
      Serial.print(";");
      */
      Serial.print(bme.temperature+273.15);
      Serial.print(";");
      Serial.print(bme.pressure);
      Serial.print(";");
      Serial.print(bme.humidity);
      Serial.print(";");
      Serial.println(bme.gas_resistance);


    }
    delay(100);

  }

}