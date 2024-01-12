#include <Ethernet.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//Switch
int SwitchPin = 8;
int DirPin = 2;
int StepPin = 3;
int SwitchStatus, DirStatus, StepStatus;
int stepsPerRevolution=200;
int WaitTimeSpeed=1;
float xPos;
bool CalDone=0;
int StepForMm=200;
int AnalogSwitchPin = A1;
//int stepDist=0.00503

float ZeroCal(){
  // Set the spinning direction clockwise:
  digitalWrite(DirPin, HIGH);
  while (digitalRead(SwitchPin)==1 && Serial.read() != 90){
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
  }
  delay(100);
  //move a little from the switch
  float diffX=5;
  int stepsToDo = 0;
  stepsToDo = abs(diffX*StepForMm);
  digitalWrite(DirPin, LOW);
  int iter=0;
  for ( int i = 0; i < stepsToDo; i++ ) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
    if (Serial.read() == 90) break;
    iter++;
  }
  return 0;
}

float getXPos(){

  return xPos;
}

void SetPos(float &realX, float fakeX){

  if (CalDone == 0){
    Serial.println("Calibration not done!");
    return 1;
  }
  else{
    //Serial.print("Initial position is:");
    //Serial.println(realX);
    //Serial.print("Final position is:");
    //Serial.println(fakeX);
    float diffX;
    int stepsToDo = 0;
    diffX = fakeX - realX;
    //Serial.println(diffX);
    if (fakeX > 140 || fakeX < 0){
      Serial.println("Impossible movement");
      return 1;
    }
    if ( diffX < 0) digitalWrite(DirPin, HIGH);
    else if ( diffX > 0) digitalWrite(DirPin, LOW);
    else if (diffX == 0 ) {
      Serial.println("Already in position");
      return 1;
    }
    stepsToDo = abs(diffX*StepForMm);
    Serial.println(stepsToDo);
    int iter=0;
    for ( int i = 0; i < stepsToDo; i++ ) {
      digitalWrite(StepPin, HIGH);
      delay(WaitTimeSpeed);
      digitalWrite(StepPin, LOW);
      delay(WaitTimeSpeed);
      if (Serial.read() == 90) break;
      if (digitalRead(SwitchPin) == 0) break;
      iter++;
    }
    float temp=realX;
    float nvm =((float)iter/(float)StepForMm);
    if ( diffX < 0) temp = temp - nvm;
    else if ( diffX > 0) temp = temp + nvm;
    realX=temp;
  }
}
//BME SENSOR
Adafruit_BME680 bme; // I2C
int incomingByte = 0; // for incoming serial data
void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  //Stich
  pinMode(SwitchPin, INPUT);
  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);

  //BME680
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    //while (1);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

//BME SENSOR
// the loop routine runs over and over again forever:
void loop() {
  //SwitchStatus = digitalRead(SwitchPin);   // read the input pin
  //Serial.println(SwitchStatus);
  //delay(100);
  //BME680
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    //Serial.print("Incoming Byte is: ");
    //Serial.println(incomingByte);

    //Read BME sensor (R)
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
      Serial.print(bme.temperature+273.15);
      Serial.print(";");
      Serial.print(bme.pressure);
      Serial.print(";");
      Serial.print(bme.humidity);
      Serial.print(";");
      Serial.println(bme.gas_resistance);
    }
    //CALIBRATION (C)
    if (incomingByte == 67){
      xPos=ZeroCal();
      CalDone=1;
      Serial.println("Calibration Done!");
    }
    //Position check (G)
    if (incomingByte == 71){
      Serial.print("Current position: ");
      Serial.println(getXPos());
    }
    //Ask if the calibation has been done (Y)
    if (incomingByte == 89){
      Serial.print("Calibration: ");
      Serial.println(CalDone);
    }
    //Who am I (W)
    if (incomingByte == 87){
      Serial.print("KEG");
    }
    //Movement (P then number in mm)
    if (incomingByte == 80){
      float pos=0;
      Serial.println("Write position");
      while( pos == 0){
        pos=Serial.readString().toFloat();
      }
      Serial.print("Selected Position mm should be: ");
      Serial.println(pos);
      Serial.print("Initial position in mm is: ");
      Serial.println(xPos);
      SetPos(xPos, pos);
      Serial.print("Actual Position in Mm is: ");
      Serial.println(xPos);
      //send position fucntion
    }
    delay(100);
  }
    //Serial.println(analogRead(AnalogSwitchPin));
    //Serial.println(digitalRead(SwitchPin));
    //delay(100);
}