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

// Auto-stop: brake the flywheel when |error| exceeds TIP_OFF_ANGLE and
// resume balance control within TIP_REARM_ANGLE (degrees).
const float TIP_OFF_ANGLE = 15;
const float TIP_REARM_ANGLE = 5.0;
const float TIP_BRAKE_GAIN = 100.0;

// With the MA5 motor and encoder directions, positive flywheel speed is
// bike-left
const float FLYWHEEL_EFFORT_SIGN = -1.0;

const unsigned long INPUT_IDLE_MS = 150;
const int MAX_VALUE_LENGTH = 15;

// IMU objects from the BNO055 example.
bno055_t myBNO;
bno055_euler myEuler;

// Menu states: 0 = print menu, 1 = main menu, 2 = set point entry,
// 3 = kp entry, 4 = kd entry, 5 = kf entry, -1 = basic data,
// -2 = full debug data.
int stateMenu = 0;

// Balance state set in menu 1: 0 = off, 1 = on.
int balanceState = 0;
bool tippedOver = false;
bool tipChangePending = false;

float setPoint = 0.0;
float kp = 0.0;
float kd = 0.0;
float kf = 0.0;

float roll = 0.0;
float error = 0.0;
float previousError = 0.0;
float errorChange = 0.0;
bool hasPreviousError = false;
long flywheelSpeed = 0;

float pEffort = 0.0;
float dEffort = 0.0;
float fEffort = 0.0;
float requestedEffort = 0.0;
int appliedDuty = 0;

// save typed characters until the whole number has been entered.
char valueInput[MAX_VALUE_LENGTH + 1];
int valueInputLength = 0;
bool valueInputHasPoint = false;
bool valueInputHasDigit = false;
unsigned long lastValueInputTime = 0;

unsigned long loop1 = 0;
unsigned long loop2 = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    // wait for the serial monitor to open.
  }

  // start I2C
  Wire.begin();

  if (!controller.begin()) {
    Serial.println("Motor Carrier connection failed");
    while (true) {
      // stop here if the motor carrier cannot connect.
    }
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

  // Control loop at 100 Hz: read sensors, calculate the control effort,
  // then check the balance state and set the flywheel duty cycle.
  if (TriggerEvent(time, loop1, FREQ_100HZ)) {
    loop1 = time;

    bno055_read_euler_hrp(&myEuler);
    roll = myEuler.r / 16.0;

    // Positive flywheel velocity and requested duty are bike-left
    // clockwise when viewed from the front.
    flywheelSpeed = -FLYWHEEL_ENCODER.getCountPerSecond();

    error = setPoint - roll;
    if (!hasPreviousError) {
      previousError = error;
      hasPreviousError = true;
    }

    errorChange = previousError - error;
    previousError = error;

    // A left lean needs positive flywheel effort to push the bike right.
    pEffort = -kp * error;
    // growing left lean produces
    // positive D effort, also opposing the bike fall.
    dEffort = kd * errorChange;
    // damping opposite to the measured flywheel spin.
    fEffort = FLYWHEEL_EFFORT_SIGN * kf * flywheelSpeed;
    requestedEffort = pEffort + dEffort + fEffort;

    // use the size of the error, whether the bike leans left or right.
    float tilt = error;
    if (tilt < 0) {
      tilt = -tilt;
    }

    if (tippedOver == false) {
      if (tilt > TIP_OFF_ANGLE) {
        tippedOver = true;
        tipChangePending = true;
      }
    } else {
      if (tilt < TIP_REARM_ANGLE) {
        tippedOver = false;
        tipChangePending = true;
      }
    }

    // start with no motor power. only change this if balance is on.
    float motorEffort = 0.0;
    if (balanceState == 1) {
      if (tippedOver == false) {
        motorEffort = requestedEffort;
      } else if (flywheelSpeed != 0) {
        // brake in the direction opposite to the flywheel's spin.
        motorEffort = -TIP_BRAKE_GAIN * flywheelSpeed;
      }
    }

    // keep the duty between -100 and 100, then round to a whole number.
    if (motorEffort > 100) {
      motorEffort = 100;
    } else if (motorEffort < -100) {
      motorEffort = -100;
    }
    appliedDuty = round(motorEffort);
    FLYWHEEL_MOTOR.setDuty(-appliedDuty);
    controller.ping();
  }

  // Print the menu or data and process input at 10 Hz.
  if (TriggerEvent(time, loop2, FREQ_10HZ)) {
    loop2 = time;

    if (tipChangePending) {
      tipChangePending = false;
      if (tippedOver) {
        Serial.println("Tip detected: braking flywheel");
      } else {
        Serial.println("Upright: flywheel re-armed");
      }
    }

    if (stateMenu == 0) {
      PrintMenu();
      stateMenu = 1;
    } else if (stateMenu == -1) {
      PrintData();
    } else if (stateMenu == -2) {
      PrintDebugData();
    }

    if (Serial.available() > 0) {
      if (stateMenu == 1) {
        char input = Serial.read();
        HandleMenuKey(input);
      } else if (stateMenu >= 2 && stateMenu <= 5) {
        ReadValueInput();
      } else if (stateMenu == -1 || stateMenu == -2) {
        // A key strike leaves print mode. 'p' and 'm' print the main menu;
        // line endings are ignored so Serial Monitor can keep streaming data.
        // Other keys use menu 1 (e.g. '0' stops balance, 'e' shows debug).
        char input = Serial.read();
        if (input != '\r' && input != '\n') {
          stateMenu = 1;
          if (input == 'p' || input == 'm') {
            stateMenu = 0;
          } else {
            HandleMenuKey(input);
          }
        }
      }
    }

    // also accept numbers when the serial monitor sends no newline.
    if (stateMenu >= 2 && stateMenu <= 5) {
      if (valueInputLength > 0) {
        if (millis() - lastValueInputTime >= INPUT_IDLE_MS) {
          SetValueFromInput();
        }
      }
    }
  }
}

// Menu 1: handles key strikes, turns balance on/off, and redirects to
// menus 2 to 5 or either print data mode.
void HandleMenuKey(char input) {
  if (input == '0') {
    balanceState = 0;
    appliedDuty = 0;
    FLYWHEEL_MOTOR.setDuty(0);
    Serial.println("Balance off");
  } else if (input == '1') {
    balanceState = 1;
    if (tippedOver) {
      Serial.println("Balance on (waiting for upright)");
    } else {
      Serial.println("Balance on");
    }
  } else if (input >= '2' && input <= '5') {
    stateMenu = input - '0';
    valueInputLength = 0;
    valueInputHasPoint = false;
    valueInputHasDigit = false;
    PrintValuePrompt();
  } else if (input == 'p') {
    stateMenu = -1;
    PrintDataHeader();
  } else if (input == 'e') {
    PrintDebugMenu();
  } else if (input == 'v') {
    stateMenu = -2;
    PrintDebugDataHeader();
  } else if (input == 'm') {
    stateMenu = 0;
  } else if (input != '\r' && input != '\n') {
    Serial.println("Input parser error");
  }
}

// menus 2 to 5: read one character at a time so control can keep running.
void ReadValueInput() {
  while (Serial.available() > 0 && stateMenu >= 2 && stateMenu <= 5) {
    char input = Serial.read();
    if (input == '\r' || input == '\n') {
      if (valueInputLength > 0) {
        SetValueFromInput();
      }
    } else {
      // allow digits, a sign at the start, and one decimal point.
      bool validCharacter = false;
      if (input >= '0' && input <= '9') {
        validCharacter = true;
      } else if (input == '-' || input == '+') {
        if (valueInputLength == 0) {
          validCharacter = true;
        }
      } else if (input == '.') {
        if (valueInputHasPoint == false) {
          validCharacter = true;
        }
      }

      if (validCharacter && valueInputLength < MAX_VALUE_LENGTH) {
        valueInput[valueInputLength] = input;
        valueInputLength = valueInputLength + 1;

        if (input >= '0' && input <= '9') {
          valueInputHasDigit = true;
        } else if (input == '.') {
          valueInputHasPoint = true;
        }
        lastValueInputTime = millis();
      } else {
        RejectValueInput();
      }
    }
  }
}

void RejectValueInput() {
  valueInputLength = 0;
  valueInputHasPoint = false;
  valueInputHasDigit = false;
  ClearSerialBuffer();
  Serial.println("Invalid number; value unchanged");
  PrintValuePrompt();
}

void SetValueFromInput() {
  // a sign or decimal point by itself is not a number.
  if (valueInputHasDigit == false) {
    valueInputLength = 0;
    valueInputHasPoint = false;
    valueInputHasDigit = false;
    Serial.println("Invalid number; value unchanged");
    PrintValuePrompt();
    return;
  }

  // '\0' marks the end of the text. atof converts that text to a number.
  valueInput[valueInputLength] = '\0';
  float enteredValue = atof(valueInput);
  valueInputLength = 0;
  valueInputHasPoint = false;
  valueInputHasDigit = false;

  if (stateMenu == 2) {
    setPoint = enteredValue;
    hasPreviousError = false;
    Serial.print("Set point set to ");
  } else if (stateMenu == 3) {
    kp = enteredValue;
    Serial.print("kp set to ");
  } else if (stateMenu == 4) {
    kd = enteredValue;
    Serial.print("kd set to ");
  } else if (stateMenu == 5) {
    kf = enteredValue;
    Serial.print("kf set to ");
  }
  Serial.println(enteredValue, 4);
  stateMenu = 1;
}

void PrintValuePrompt() {
  if (stateMenu == 2) {
    Serial.println("Enter the desired set point (degrees):");
  } else if (stateMenu == 3) {
    Serial.println("Enter kp:");
  } else if (stateMenu == 4) {
    Serial.println("Enter kd:");
  } else if (stateMenu == 5) {
    Serial.println("Enter kf:");
  }
}

void PrintMenu() {
  Serial.print("Balance: ");
  if (balanceState == 1) {
    Serial.print("on");
  } else {
    Serial.print("off");
  }
  if (tippedOver) {
    Serial.println(" (tipped over)");
  } else {
    Serial.println();
  }
  Serial.print("Set point: ");
  Serial.print(setPoint, 2);
  Serial.print("  kp: ");
  Serial.print(kp, 4);
  Serial.print("  kd: ");
  Serial.print(kd, 4);
  Serial.print("  kf: ");
  Serial.println(kf, 4);
  Serial.println("0 off | 1 on | 2 set point");
  Serial.println("3 kp | 4 kd | 5 kf");
  Serial.println("p basic data | e extra debug | m menu");
}

void PrintDebugMenu() {
  Serial.println("Extra debug:");
  Serial.println("v: full data (error, efforts, gains, balance)");
  Serial.println("p: basic data | m: main menu | 0: balance off");
}

void PrintDataHeader() {
  Serial.println("roll\tsetPt\tflySpd\tduty\ttip");
}

void PrintData() {
  Serial.print(roll, 2);
  Serial.print('\t');
  Serial.print(setPoint, 2);
  Serial.print('\t');
  Serial.print(flywheelSpeed);
  Serial.print('\t');
  Serial.print(appliedDuty);
  Serial.print('\t');
  if (tippedOver) {
    Serial.println(1);
  } else {
    Serial.println(0);
  }
}

void PrintDebugDataHeader() {
  Serial.println("roll\tsetPt\terror\tflySpd\tpEff\tdEff\tfEff\t"
                 "effort\tduty\tbal\ttip\tkp\tkd\tkf");
}

void PrintDebugData() {
  Serial.print(roll, 2);
  Serial.print('\t');
  Serial.print(setPoint, 2);
  Serial.print('\t');
  Serial.print(error, 2);
  Serial.print('\t');
  Serial.print(flywheelSpeed);
  Serial.print('\t');
  Serial.print(pEffort, 2);
  Serial.print('\t');
  Serial.print(dEffort, 2);
  Serial.print('\t');
  Serial.print(fEffort, 2);
  Serial.print('\t');
  Serial.print(requestedEffort, 2);
  Serial.print('\t');
  Serial.print(appliedDuty);
  Serial.print('\t');
  Serial.print(balanceState);
  Serial.print('\t');
  if (tippedOver) {
    Serial.print(1);
  } else {
    Serial.print(0);
  }
  Serial.print('\t');
  Serial.print(kp, 4);
  Serial.print('\t');
  Serial.print(kd, 4);
  Serial.print('\t');
  Serial.println(kf, 4);
}

void ClearSerialBuffer() {
  while (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '\r' || input == '\n') {
      break;
    }
  }
}

bool TriggerEvent(unsigned long t1, unsigned long t2, int desFreq) {
  // micros() counts microseconds, so one second is 1,000,000 counts.
  unsigned long timeBetweenEvents = 1000000UL / desFreq;
  if (t1 - t2 >= timeBetweenEvents) {
    return true;
  } else {
    return false;
  }
}
