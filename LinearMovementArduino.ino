#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//Function to software reset
void(* Reset)(void) = 0;

// Pin definitions for the stepper motor and switch
int SwitchPin = 8; // Pin connected to the switch
int DirPin = 2;    // Direction control pin for stepper motor
int StepPin = 3;   // Step control pin for stepper motor
int enablePin = 5;    // ENABLE pin

// Variables to store the status of the switch and motor direction and step
int SwitchStatus, DirStatus, StepStatus;

// Stepper motor configuration
int stepsPerRevolution = 200; // Number of steps per full revolution of motor
int WaitTimeSpeed = 1;        // Delay time for motor control, in milliseconds
float xPos;                   // Current position of the motor
bool CalDone = 0;             // Flag indicating if calibration is done
int StepForMm = 200;          // Number of steps per millimeter of movement

// Pin for reading analog values from a switch (if used)
int AnalogSwitchPin = A1;

// Function to calibrate the zero position of the stepper motor
float ZeroCal() {
  // Set the spinning direction clockwise
  digitalWrite(DirPin, HIGH);

  // Move motor until the switch is activated or a 'Z' (ASCII 90) is received on Serial
  while (digitalRead(SwitchPin) == 1 && Serial.read() != 90) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
  }
  delay(100);

  // Move motor away from the switch to a specified offset
  float diffX = 4; // Offset distance in mm
  int stepsToDo = abs(diffX * StepForMm); // Convert offset to steps
  digitalWrite(DirPin, LOW); // Change direction
  for (int i = 0; i < stepsToDo; i++) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
    if (Serial.read() == 90) break; // Stop if 'Z' is received
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
  else (fakeX == 0){
    Serial.println("To go to 0 position use Cal function instead")
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
  for (int i = 0; i < stepsToDo; i++) {
    digitalWrite(StepPin, HIGH);
    delay(WaitTimeSpeed);
    digitalWrite(StepPin, LOW);
    delay(WaitTimeSpeed);
    if (Serial.read() == 90) break; // Stop if 'Z' is received
    if (digitalRead(SwitchPin) == 0) break; // Stop if switch is activated
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
  pinMode(enablePin, OUTPUT);

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

  // INITIALLY disable the driver
  digitalWrite(enablePin, HIGH);
}

// Main loop function, runs repeatedly
void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // Read the incoming byte

    // Uncomment for degub about message received
    Serial.print("Command received is: ");
    Serial.println(incomingByte);
    delay(100);

    //Software reset the board
    if (incomingByte == 75) { // 'K' command
      Serial.println("Resetting Arduinio...");
      delay(100);
      Reset();
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
    }

    // Handle calibration
    if (incomingByte == 67) { // 'C' command
      // Initially enable the driver
      digitalWrite(enablePin, LOW);
      delay(10);
      Serial.println("Calibrating...");
      xPos = ZeroCal();
      CalDone = 1;
      Serial.println("Calibration Done!");
      // Disable the driver
      digitalWrite(enablePin, HIGH);
      delay(10);
    }

    // Position check
    if (incomingByte == 71) { // 'G' command
      Serial.print("Current position: ");
      Serial.println(getXPos());
    }

    // Check if calibration has been done
    if (incomingByte == 89) { // 'Y' command
      Serial.print("Calibration: ");
      Serial.println(CalDone);
    }

    // Identify device
    if (incomingByte == 87) { // 'W' command
      Serial.println("KEG");
    }

    // Get the ENABLE PIN status
    if (incomingByte == 69) { // 'E' command
      Serial.print("Enable Pin status: ");
      Serial.println(digitalRead(enablePin));
    }

    // Movement command
    if (incomingByte == 80) { // 'P' command
      if (CalDone == 0) {
        Serial.println("Calibration not done, I will not move!");
        return;
      }
      else{
        // Initially enable the driver
        digitalWrite(enablePin, LOW);
        delay(10);
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
        Serial.println("Moving...");
        SetPos(xPos, pos); // Move to the specified position
        Serial.print("Actual Position in Mm is: ");
        Serial.println(xPos);
        // Disable the driver
        digitalWrite(enablePin, HIGH);
        delay(10);
      }
      // Add a small delay for stability
      delay(100);
    }
  }

  // Uncomment for degub about the Switch
  //Serial.println(analogRead(AnalogSwitchPin));
  //Serial.println(digitalRead(SwitchPin));
  //delay(100);


}