#include <Wire.h>
#include <ArduinoMotorCarrier.h>
#include <stdlib.h>
#include "BNO055_support.h"

// Motor and encoder channels recorded in the MA5 hardware notes.
// Recheck the wiring and directions on the bike before balance control.
#define FLYWHEEL_MOTOR M3
#define FLYWHEEL_ENCODER encoder1


#define FREQ_100HZ (100)
#define FREQ_10HZ (10)

// IMU objects from the BNO055 example.
bno055_t myBNO;
bno055_euler myEuler;
bno055_gyro myGyro;

// Motor and menu states: 0 = menu, 1 = main menu, 2 = duty entry,
// -1 = print sensor data.
int mtrState = 0;
int stateMenu = 0;

int dutyCycle = 0;
char dutyInput[12];
byte dutyInputLength = 0;
unsigned long lastDutyInputTime = 0;
float roll = 0.0;
float rollDeriv = 0.0;
int flywheelDeriv = 0;

unsigned long loop1 = 0;
unsigned long loop2 = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial);

  // start I2C
  Wire.begin();

  if (!controller.begin()) {
    Serial.println("Motor Carrier connection failed");
    while (true);
  }
  Wire.setClock(400000);


  // set all duty cycles to 0
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);

  //establish imu

  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(1);
}

void loop() {
  unsigned long time = micros();

  // Read sensors and update the flywheel motor at 100 Hz.
  if (TriggerEvent(time, loop1, FREQ_100HZ)) {
    loop1 = time;

    bno055_read_euler_hrp(&myEuler);
    roll = float(myEuler.r) / 16.0;

    bno055_read_gyro_xyz(&myGyro);
    rollDeriv = -float(myGyro.y) / 16.0;

    // Positive flywheel velocity and requested duty are bike-left
    // (clockwise when viewed from the front).
    flywheelDeriv = -FLYWHEEL_ENCODER.getCountPerSecond();

    if (mtrState == 1) {
      FLYWHEEL_MOTOR.setDuty(-dutyCycle);
    } else {
      FLYWHEEL_MOTOR.setDuty(0);
    }
    controller.ping();
  }

  // Print the menu or sensor data and process input at 10 Hz.
  if (TriggerEvent(time, loop2, FREQ_10HZ)) {
    loop2 = time;

    if (stateMenu == 0) {
      PrintMenu();
      stateMenu = 1;
    } else if (stateMenu == -1) {
      Serial.print(roll);
      Serial.print('\t');
      Serial.print(rollDeriv);
      Serial.print('\t');
      Serial.print(flywheelDeriv);
      Serial.print('\t');
      Serial.println(mtrState == 1 ? dutyCycle : 0);
    }

    if (Serial.available() > 0) {
      if (stateMenu == 1) {
        char input = Serial.read();
        ClearSerialBuffer();

        if (input == '0') {
          mtrState = 0;
          FLYWHEEL_MOTOR.setDuty(0);
        } else if (input == '1') {
          mtrState = 1;
        } else if (input == 'p') {
          stateMenu = -1;
        } else if (input == 'm') {
          stateMenu = 0;
        } else if (input == 'd') {
          stateMenu = 2;
          dutyInputLength = 0;
          Serial.println("Enter the motor duty cycle (-100 to 100):");
        } else if (input != '\r' && input != '\n') {
          Serial.println("Input parser error");
        }
      } else if (stateMenu == 2) {
        while (Serial.available() > 0 && stateMenu == 2) {
          char input = Serial.read();
          if (input == '\r' || input == '\n') {
            if (dutyInputLength > 0) {
              SetDutyCycleFromInput();
            }
          } else if ((input >= '0' && input <= '9') ||
                     ((input == '-' || input == '+') && dutyInputLength == 0)) {
            if (dutyInputLength < sizeof(dutyInput) - 1) {
              dutyInput[dutyInputLength++] = input;
              lastDutyInputTime = millis();
            } else {
              dutyInputLength = 0;
              ClearSerialBuffer();
              Serial.println("Enter an integer from -100 to 100:");
            }
          } else {
            dutyInputLength = 0;
            ClearSerialBuffer();
            Serial.println("Enter an integer from -100 to 100:");
          }
        }
      } else if (stateMenu == -1) {
        char input = Serial.read();
        ClearSerialBuffer();
        stateMenu = 1;
        if (input == 'd') {
          stateMenu = 2;
          dutyInputLength = 0;
          Serial.println("Enter the motor duty cycle (-100 to 100):");
        } else if (input == 'm') {
          stateMenu = 0;
        }
      }
    }

    if (stateMenu == 2 && dutyInputLength > 0 &&
        millis() - lastDutyInputTime >= 150) {
      SetDutyCycleFromInput();
    }
  }
}

void SetDutyCycleFromInput() {
  dutyInput[dutyInputLength] = '\0';
  char *end;
  long enteredDuty = strtol(dutyInput, &end, 10);
  dutyInputLength = 0;

  if (end == dutyInput || *end != '\0') {
    Serial.println("Enter an integer from -100 to 100:");
    return;
  }

  dutyCycle = constrain(enteredDuty, -100L, 100L);
  stateMenu = 1;
  Serial.print("Motor duty cycle set to ");
  Serial.println(dutyCycle);
}

void PrintMenu() {
  Serial.print("Motor state: ");
  Serial.println(mtrState);
  Serial.print("Motor duty cycle: ");
  Serial.println(dutyCycle);
  Serial.println("0: stop the motor");
  Serial.println("1: start the motor");
  Serial.println("m: print the menu");
  Serial.println("d: set motor duty cycle (-100 to 100; positive = bike-left)");
  Serial.println("p: print sensor data and motor command");
}

void ClearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

bool TriggerEvent(unsigned long t1, unsigned long t2, int desFreq) {
  return (t1 - t2) >= (1000000UL / desFreq);
}
