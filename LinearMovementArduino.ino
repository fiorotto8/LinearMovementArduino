#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Servo.h>

//Function to software reset
void(* Reset)(void) = 0;

// Pin definitions for the stepper motor and switch
const int SwitchPin = 4; // Pin connected to the switch
const int DirPin = 2;    // Direction control pin for stepper motor
const int StepPin = 3;   // Step control pin for stepper motor
//int enablePin = 5;    // ENABLE pin
const int relay1Pin = 10;    // Relay 1 pin
const int relay2Pin = 11;    // Relay 2 pin

Servo myServo;

bool relayStatus;

// Variables to store the status of the switch and motor direction and step
int SwitchStatus, DirStatus, StepStatus;

// Stepper motor configuration
const int stepsPerRevolution = 200; // Number of steps per full revolution of motor
const int WaitTimeSpeed = 1;        // Delay time for motor control, in milliseconds
const int StepForMm = 200;          // Number of steps per millimeter of movement
float xPos;                   // Current position of the motor
bool CalDone = 0;             // Flag indicating if calibration is done


// Pin for reading analog values from a switch (if used)
const int AnalogSwitchPin = A1;

// Function to excite the relays i.e. powering ON the AC/DC converter
bool ExciteRelays(){
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  return true;
}

// Function to deexcite the relays i.e. powering OFF the AC/DC converter
bool DexciteRelays(){
  digitalWrite(relay1Pin, HIGH);
  digitalWrite(relay2Pin, HIGH);
  return false;
}

// Function to calibrate the zero position of the stepper motor
float ZeroCal() {
  // Set the spinning direction clockwise
  digitalWrite(DirPin, HIGH);

  bool digi_zero;
  int anal_zero;
  // Move motor until the switch is activated or a 'Z' (ASCII 90) is received on Serial
  //while (digitalRead(SwitchPin) == 1 && Serial.read() != 90) {
  while (true) {

    digi_zero = digitalRead(SwitchPin);
    anal_zero = analogRead(AnalogSwitchPin);
    //Serial.println(digi_zero);
    //CSerial.println(anal_zero);
    int incoming = Serial.read();
    if (incoming == 90){ // Stop if 'Z' is received
       Serial.println("Z received stopping calibration..."); 
       break;
    }
      // If we got *some* other character (not -1), ignore it and dump the buffer
    else if (incoming != -1) {
      // Dump any extra characters waiting in the buffer
      while (Serial.available() > 0) {
        Serial.read();
      }
      Serial.println("Ignoring unexpected message, dumping buffer...");
    } 
    //if (digi_zero == 0){ // Stop if switch is activated
    //  Serial.println("Switch Pin hit endpoint found...");
    //  break;
    //} 

    if (anal_zero < 500){ // Stop if switch is activated
      Serial.println("Switch Pin hit endpoint found...");
      break;
    } 

    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
  }
  delay(100);

  // Move motor away from the switch to a specified offset
  float diffX = 5.5; // Offset distance in mm
  int stepsToDo = abs(diffX * StepForMm); // Convert offset to steps
  digitalWrite(DirPin, LOW); // Change direction
  for (int i = 0; i < stepsToDo; i++) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
  }
  return 0;
}

// Function to return the current position of the motor
float getXPos() {
  return xPos;
}

// Function to move the motor to a specified position
void SetPos(float &realX, float fakeX) {
  float diffX; // Difference between current and target position
  int stepsToDo = 0; // Number of steps to move
  diffX = fakeX - realX; // Calculate the difference in position

  // Check for boundary conditions
  if (fakeX > 110 || fakeX < 0) {
    Serial.println("Impossible movement");
    return;
  }
  else if (fakeX == 0){
    Serial.println("To go to 0 position use Cal function instead");
    return;
  }

  // Set motor direction based on the position difference
  if (diffX < 0) digitalWrite(DirPin, HIGH);
  else if (diffX > 0) digitalWrite(DirPin, LOW);
  else if (diffX == 0) {
    Serial.println("Already in position");
    return;
  }

  stepsToDo = abs(diffX * StepForMm); // Convert position difference to steps
  //Serial.println(stepsToDo);

  // Move the motor the calculated number of steps
  int iter = 0;
  /*
  Serial.println("stepsToDo - iters");
  Serial.print(stepsToDo);
  Serial.print(" ");
  Serial.println(iter);
  */
  for (int i = 0; i < stepsToDo; i++) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);

    int anal_pos = analogRead(AnalogSwitchPin);

    int incoming = Serial.read();
    if (incoming == 90){ // Stop if 'Z' is received
       Serial.println("Z received stopping movement..."); 
       break;
    }
    // If we got *some* other character (not -1), ignore it and dump the buffer
    else if (incoming != -1) {
      // Dump any extra characters waiting in the buffer
      while (Serial.available() > 0) {
        Serial.read();
      }
      Serial.println("Ignoring unexpected message, dumping buffer...");
    }
    /*
    if (digitalRead(SwitchPin) == 0){ // Stop if switch is activated
      Serial.println("Switch Pin hit stopping movement...");
      break;
    } 
    */

    if (anal_pos < 500 ){ // Stop if switch is activated
      Serial.println("Switch Pin hit stopping movement...");
      break;
    }
    /*
    Serial.println("stepsToDo - iters");
    Serial.print(stepsToDo);
    Serial.print(" ");
    Serial.println(iter);
    */
    iter++;
  }

  // Update the real position of the motor
  float temp = realX;
  float nvm = ((float)iter / (float)StepForMm);
  if (diffX < 0) temp = temp - nvm;
  else if (diffX > 0) temp = temp + nvm;
  realX = temp;
}

// Initialization of BME680 sensor
Adafruit_BME680 bme; // I2C
int incomingByte = 0; // Variable to store incoming serial data


// Setup function, runs once at startup
void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud rate

  // Setup pins for the stepper motor and switch
  pinMode(SwitchPin, INPUT);
  pinMode(DirPin, OUTPUT);
  pinMode(StepPin, OUTPUT);
  //pinMode(enablePin, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);

  // Initialize BME680 sensor
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    //while (1); // Uncomment to halt if sensor is not found
  }

  // Set up oversampling and filter initialization for BME680
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // Set heater temperature and duration

  //Deexcite realys!
  relayStatus=DexciteRelays();
  // INITIALLY disable the driver
  //digitalWrite(enablePin, HIGH);

}

// Main loop function, runs repeatedly
void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Read the incoming byte

    // Uncomment for degub about message received
    //Serial.print("Command received is: ");
    //Serial.println(incomingByte);
    //delay(100);

    //Move calibration source seromotor
    if (incomingByte == 76) { // 'L' command
      int anglePos=0;
      Serial.println("Write Servo Position (1, 2, 3 or 4)");
      while (anglePos==0){
        anglePos = Serial.readString().toInt();
      }
      int angle=-1;
      if (anglePos==1){angle=0;}
      else if (anglePos==2){angle=60;}
      else if (anglePos==3){angle=125;}
      else if (anglePos==4){angle=270;}
      else {
        Serial.println("Insert a valid position");
        return;
      }
      myServo.attach(9, 500, 2500);
      myServo.write(angle);
      delay(1000);
      Serial.print("Servo moved to Pos");
      Serial.print(anglePos);
      Serial.print(" i.e. angle ");
      Serial.println(angle);
      myServo.detach();
    }

    //Software reset the board
    if (incomingByte == 75) { // 'K' command
      Serial.println("Resetting Arduinio...");
      delay(100);
      Reset();
      Serial.println("PANDA");
    }    

    // Handle BME sensor reading
    if (incomingByte == 82) { // 'R' command
      unsigned long endTime = bme.beginReading();
      if (endTime == 0) {
        Serial.println(F("Failed to begin reading :("));
        return;
      }
      if (!bme.endReading()) {
        Serial.println(F("Failed to complete reading :("));
        return;
      }
      // Output sensor readings
      Serial.print(bme.temperature + 273.15); // Convert to Kelvin
      Serial.print(";");
      Serial.print(bme.pressure);
      Serial.print(";");
      Serial.print(bme.humidity);
      Serial.print(";");
      Serial.println(bme.gas_resistance);
      Serial.println("PANDA");
    }

    // Handle calibration
    if (incomingByte == 67) { // 'C' command
      // Initially ecite relays and enable the driver
      Serial.println("Excite relays...");
      relayStatus=ExciteRelays();
      delay(2000);
      //digitalWrite(enablePin, LOW);
      //delay(2000);
      //start calibtion
      Serial.println("Calibrating...");
      xPos = ZeroCal();
      CalDone = 1;
      // Disable the driver and deectire realys
      Serial.println("Dexcite relays...");
      delay(100);
      relayStatus=DexciteRelays();
      delay(2000);
      //digitalWrite(enablePin, HIGH);
      //delay(2000);
      Serial.println("Calibration Done!");
      Serial.println("PANDA");
    }

    // Position check
    if (incomingByte == 71) { // 'G' command
      Serial.print("Current position: ");
      Serial.println(getXPos());
      Serial.println("PANDA");
    }

    // Realy Status check
    if (incomingByte == 77) { // 'M' command
      Serial.print("Relays excited: ");
      Serial.println(relayStatus);
      Serial.println("PANDA");
    }

    // Check if calibration has been done
    if (incomingByte == 89) { // 'Y' command
      Serial.print("Calibration: ");
      Serial.println(CalDone);
      Serial.println("PANDA");
    }

    // Identify device
    if (incomingByte == 87) { // 'W' command
      Serial.println("KEG");
      Serial.println("PANDA");
    }

    // Get Relays Status
    if (incomingByte == 83) { // 'S' command
      Serial.print("Relays are: ");
      Serial.println(relayStatus);
      Serial.println("PANDA");
    }

    // Excite relays
    if (incomingByte == 84) { // 'T' command
      Serial.print("Relays excited ");
      Serial.println(relayStatus=ExciteRelays());
      Serial.println("PANDA");
    }

    // Dexcite relays
    if (incomingByte == 85) { // 'U' command
      Serial.print("Relays deexcited ");
      Serial.println(relayStatus=DexciteRelays());
      Serial.println("PANDA");
    }

    // Movement command
    if (incomingByte == 80) { // 'P' command
      if (CalDone == 0) {
        Serial.println("Calibration not done, I will not move!");
        return;
      }
      else{
        // start movement
        float pos = 0;
        Serial.println("Write position");
        // Wait for a valid position input
        while (pos == 0) {
          pos = Serial.readString().toFloat();
        }
        Serial.print("Selected Position mm should be: ");
        Serial.println(pos);
        Serial.print("Initial position in mm is: ");
        Serial.println(xPos);
        // Initially ecite relays and enable the driver
        Serial.println("Excite relays...");
        relayStatus=ExciteRelays();
        delay(2000);
        //digitalWrite(enablePin, LOW);
        //delay(3000);
        //start moving
        Serial.println("Moving...");
        SetPos(xPos, pos); // Move to the specified position
        // Disable the driver and deectire realys
        Serial.println("Dexcite relays...");
        delay(100);
        relayStatus=DexciteRelays();
        delay(2000);
        //digitalWrite(enablePin, HIGH);
        //delay(2000);
        Serial.print("Actual Position in Mm is: ");
        Serial.println(getXPos());
      }
      // Add a small delay for stability
      delay(100);
      Serial.println("PANDA");
    }
  }

  // Uncomment for degub about the Switch
  //Serial.println(analogRead(AnalogSwitchPin));
  //Serial.println(digitalRead(SwitchPin));
  //delay(100);


}
