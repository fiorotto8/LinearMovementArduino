# Moving stage for Source and TPH sensor

![alt text](Schetch_bb.png)
NB: the image of the AA battery pack is referred to the 24V DC power supply!

## Basic instructions
Send a byte through serial port to Arduino

- **R**: Arduino replies with *temperature;pressure;humidity;VOC* [K,Pa,%,$\Omega$]
- **G**: Get the position of the stage with respect to the *zero*. Possible only after calibration
- **C**: do the calibration and stays to the *zero* position
- **P**: Set the position in mm from the *zero*. After that a prompt will ask you the position in mm. Not working without calibration. It is not possible to go to *zero* with this function, use the calibration procedure.
- **Z**: To send during any movement to stop it prematurely
- **W**: To return the detector where the Arduino is connected (KEG or MANGOlino)
- **Y**: Return if the calibration has already been done
- **K**: Reset the Arduino, equivalent to click the reset button
- **T**: Excite the Relays to connect the Motor power supply to the 220V
- **U**: Dexcite the Relays to disconnect the Motor power supply to the 220V
- **S**: Get Relays status
- **L**: Set the Source holder Servomotor position. After that a prompt will ask you the position: 4 positions are possible where pos1 correspond to angle 0° and pos4 to angle 270°

The Arduino will print a lot of stuff on the Serial, however only when a return code, *E followed b a number* is given an operaation is finished:

- **E0**: no error, operation completed succesfully
- **E1**: "Calibration not done, I will not move!" if the P command is sent before the calibration
- **E2**: Calibratioin interrupted by Z command
- **E3**: During servo positioning a wrong poition was given
- **E4**: Timeout during servo positioning
- **E5**: Timeout during stepper positioning
- **E6**: Impossible movement for the stepper, negative position of greater than 110
- **E7**: P0 is not allowed use Calibration instead
- **E8**: position movement stopped by Z command
- **E9**: 'Z' command received when no operation was ongoing

## TPH sensor

The sensor is a BME 680 that measure Temperature, pressure, humidity and VOC (volatile organic compounds).
Arduino talks to it via I2C. the sensor requires +5V and GROUND. **SDA and SCL are respectively connected to A4 and A5 analog input.**
| Wire Name      | Pin On Arduino
| :---:        |    :----:   |
| VCC      | Anywhere 5V       |
| GND   | Anywhere GND        |
| SDA   | A4        |
| SCL   | A5        |

## Moving stage

The stage is moving via a bipolar stepper motor. A switch on one edge of the guide is used to get the *zero* position via calibration procedure.
The Switch is read by A1 analog Pin and it triggers when the reading is less than 500.
The motor driver A4988 should be connected both to 5V and the 24V DC power supply.

Here the mapping of the cable from the motor to the controller.
| Name on A4988      | Other name | Color of the wire     |
| :---:        |    :----:   |          :---: |
| 1B      | A+       | Black   |
| 1A   | A-        | <span style="color:green">Green</span>      |
| 2A   | B-        | <span style="color:blue">Blue</span>      |
| 2B   | B+        | <span style="color:red">Red</span>      |

Remember to connect in parallel to the 24V a 100$\mu$F capacitor to avoid to burn the controller.
We selected 1.7 k$\Omega$ for the resistance for the switch.

A recent feature is the adding of 2 relays that remove the power from the motor power supply to avoid noise to the PMT.

## Mapping of the 15pin parallel cable for the Vessel feedthroug

To enter in the vacuum vessel we used some connector with 15 pins. Here the mapping:

| Pin Number      | Connection |
| :---:        |    :----:   |
| 1      | <span style="color:green">1A</span>       |
| 2      | <span style="color:blue">2A</span>       |
| 3      | 1B       |
| 4      | <span style="color:red">2B</span>       |
| 5      | Switch -       |
| 6      | Switch Ground       |
| 7      | Switch +       |
| 8      | NC      |
| 9      | VCC 5V (Servo)       |
| 10      | GND (Servo)       |
| 11     | Servo Controller       |
| 12      |  GND (BME 680)      |
| 13      | VCC 5V (BME680)       |
| 14      | SDA       |
| 15      | SCL       |