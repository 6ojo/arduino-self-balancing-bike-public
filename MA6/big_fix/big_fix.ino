#include <Wire.h>
#include <ArduinoMotorCarrier.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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
// Stronger than the normal flywheel damping; tapers as speed approaches zero.
const float TIP_BRAKE_GAIN = 50.0;

// With the MA5 motor and encoder directions, positive flywheel speed is
// bike-left, so the damping effort must command the opposite direction.
const float FLYWHEEL_EFFORT_SIGN = -1.0;

const unsigned long INPUT_IDLE_MS = 150;
const int MAX_VALUE_LENGTH = 15;
const int MAX_INPUT_CHARS_PER_UPDATE = 16;

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
bool sensorFaultLatched = false;
bool communicationFaultMessagePending = false;
bool bnoCallbackFailed = false;
bool imuInitializationSucceeded = false;
bool validSensorSample = false;

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
bool valueInputInvalid = false;
bool discardingCanceledValue = false;
unsigned long lastValueInputTime = 0;

unsigned long loop1 = 0;
unsigned long loop2 = 0;

int CheckedBNORead(unsigned char devAddr, unsigned char regAddr,
                   unsigned char *regData, unsigned char count);
int CheckedBNOWrite(unsigned char devAddr, unsigned char regAddr,
                    unsigned char *regData, unsigned char count);
bool InitializeBNO();
bool ReadEncoderSpeed(long *speed);
void LatchCommunicationFault();

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

  // Establish the IMU, then use callbacks that report incomplete I2C reads.
  imuInitializationSucceeded = InitializeBNO();
  if (imuInitializationSucceeded == false) {
    LatchCommunicationFault();
    FLYWHEEL_MOTOR.setDuty(0);
  }
}

void loop() {
  unsigned long time = micros();

  // Control loop at 100 Hz: read sensors, calculate the control effort,
  // then check the balance state and set the flywheel duty cycle.
  if (TriggerEvent(time, loop1, FREQ_100HZ)) {
    loop1 = time;

    if (imuInitializationSucceeded == false) {
      balanceState = 0;
      appliedDuty = 0;
      FLYWHEEL_MOTOR.setDuty(0);
    } else {
      bnoCallbackFailed = false;
      int imuStatus = bno055_read_euler_r(&myEuler.r);
      bool imuReadValid = imuStatus == SUCCESS && bnoCallbackFailed == false;
      long newFlywheelSpeed = 0;
      bool encoderReadValid = ReadEncoderSpeed(&newFlywheelSpeed);

      if (imuReadValid == false || encoderReadValid == false) {
        LatchCommunicationFault();
        FLYWHEEL_MOTOR.setDuty(0);
      } else {
        // Fresh valid reads clear the communication gate, but not balanceState.
        // The user must explicitly choose menu key 1 to resume balancing.
        sensorFaultLatched = false;
        validSensorSample = true;
        roll = myEuler.r / 16.0f;
        // Positive flywheel velocity and requested duty are bike-left
        // clockwise when viewed from the front.
        flywheelSpeed = -newFlywheelSpeed;

        error = setPoint - roll;
        if (!hasPreviousError) {
          previousError = error;
          hasPreviousError = true;
        }
        // MA6 defines error change as previous minus current error.
        errorChange = previousError - error;
        previousError = error;

        // a left lean needs a
        // positive flywheel effort to push the bike right.
        pEffort = -kp * error;
        // With previous-minus-current error, a growing left lean produces
        // positive D effort, also opposing the bike's falling motion.
        dEffort = kd * errorChange;
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
      }
    }
    controller.ping();
  }

  // Process a small amount of input before printing data at 10 Hz.
  if (TriggerEvent(time, loop2, FREQ_10HZ)) {
    loop2 = time;

    ProcessSerialInput();

    if (discardingCanceledValue && Serial.available() == 0) {
      if (millis() - lastValueInputTime >= INPUT_IDLE_MS) {
        discardingCanceledValue = false;
      }
    }

    // also accept numbers when the serial monitor sends no newline.
    if (Serial.available() == 0 && stateMenu >= 2 && stateMenu <= 5 &&
        (valueInputLength > 0 || valueInputInvalid)) {
      if (millis() - lastValueInputTime >= INPUT_IDLE_MS) {
        SetValueFromInput();
      }
    }

    if (stateMenu == 0) {
      PrintMenu();
      stateMenu = 1;
    }

    if (communicationFaultMessagePending) {
      communicationFaultMessagePending = false;
      Serial.println("Sensor communication fault: balance off; press 1 after readings recover");
    }

    if (tipChangePending) {
      tipChangePending = false;
      if (tippedOver) {
        Serial.println("Tip detected: braking flywheel");
      } else {
        Serial.println("Upright: flywheel re-armed");
      }
    }

    if (stateMenu == -1) {
      PrintData();
    } else if (stateMenu == -2) {
      PrintDebugData();
    }
  }
}

// Checked read callback replacement for the installed BNO055 support library.
// Its bundled callback ignores both endTransmission() and requestFrom() results.
int CheckedBNORead(unsigned char devAddr, unsigned char regAddr,
                   unsigned char *regData, unsigned char count) {
  Wire.beginTransmission(devAddr);
  size_t registerBytes = Wire.write(regAddr);
  uint8_t transmitStatus = Wire.endTransmission();
  if (registerBytes != 1 || transmitStatus != 0) {
    bnoCallbackFailed = true;
    return ERROR1;
  }

  delayMicroseconds(150);
  uint8_t receivedCount = Wire.requestFrom((uint8_t)devAddr, (uint8_t)count);
  if (receivedCount != count) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    bnoCallbackFailed = true;
    return ERROR1;
  }

  for (unsigned char index = 0; index < count; index++) {
    if (Wire.available() <= 0) {
      bnoCallbackFailed = true;
      return ERROR1;
    }
    regData[index] = (unsigned char)Wire.read();
  }

  if (Wire.available() != 0) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    bnoCallbackFailed = true;
    return ERROR1;
  }
  return SUCCESS;
}

// Check write-byte counts and the I2C transmission result as well.
int CheckedBNOWrite(unsigned char devAddr, unsigned char regAddr,
                    unsigned char *regData, unsigned char count) {
  Wire.beginTransmission(devAddr);
  size_t writtenBytes = Wire.write(regAddr);
  for (unsigned char index = 0; index < count; index++) {
    writtenBytes += Wire.write(regData[index]);
  }
  uint8_t transmitStatus = Wire.endTransmission();
  delayMicroseconds(150);

  if (writtenBytes != (size_t)count + 1 || transmitStatus != 0) {
    bnoCallbackFailed = true;
    return ERROR1;
  }
  return SUCCESS;
}

bool InitializeBNO() {
  int initStatus = BNO_Init(&myBNO);

  // BNO_Init installs the library defaults; replace them before further reads.
  myBNO.bus_read = CheckedBNORead;
  myBNO.bus_write = CheckedBNOWrite;
  if (initStatus != SUCCESS) {
    return false;
  }

  unsigned char chipId = 0;
  bnoCallbackFailed = false;
  int chipIdStatus = bno055_read_chip_id(&chipId);
  if (chipIdStatus != SUCCESS || bnoCallbackFailed || chipId != 0xA0) {
    return false;
  }

  bnoCallbackFailed = false;
  int configStatus = bno055_set_operation_mode(OPERATION_MODE_CONFIG);
  if (configStatus != SUCCESS || bnoCallbackFailed) {
    return false;
  }
  delay(20);

  bnoCallbackFailed = false;
  int ndofStatus = bno055_set_operation_mode(OPERATION_MODE_NDOF);
  if (ndofStatus != SUCCESS || bnoCallbackFailed) {
    return false;
  }
  delay(10);

  unsigned char operationMode = OPERATION_MODE_CONFIG;
  bnoCallbackFailed = false;
  int modeReadStatus = bno055_get_operation_mode(&operationMode);
  if (modeReadStatus != SUCCESS || bnoCallbackFailed) {
    return false;
  }
  return operationMode == OPERATION_MODE_NDOF;
}

// ArduinoMotorCarrier 2.0.3 returns IRQ status plus four payload bytes.
// encoder1 is instance 0 in the installed library's global declarations.
bool ReadEncoderSpeed(long *speed) {
  Wire.beginTransmission(I2C_ADDRESS);
  size_t commandBytes = Wire.write((uint8_t)GET_COUNT_PER_SECOND_ENCODER);
  commandBytes += Wire.write((uint8_t)0);
  uint8_t transmitStatus = Wire.endTransmission();
  if (commandBytes != 2 || transmitStatus != 0) {
    return false;
  }

  uint8_t receivedCount = Wire.requestFrom((uint8_t)I2C_ADDRESS, (uint8_t)5);
  if (receivedCount != 5) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  uint8_t response[5];
  for (int index = 0; index < 5; index++) {
    if (Wire.available() <= 0) {
      return false;
    }
    response[index] = (uint8_t)Wire.read();
  }
  if (Wire.available() != 0) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  // The first byte is the carrier IRQ status, as in the installed getData().
  if (response[0] != 0) {
    controller.irq_status = response[0];
  }

  int32_t rawSpeed = 0;
  memcpy(&rawSpeed, &response[1], sizeof(rawSpeed));
  *speed = (long)rawSpeed;
  return true;
}

void LatchCommunicationFault() {
  if (sensorFaultLatched == false) {
    communicationFaultMessagePending = true;
  }
  sensorFaultLatched = true;
  validSensorSample = false;
  hasPreviousError = false;
  balanceState = 0;
  appliedDuty = 0;
}

// Menu 1: handles key strikes, turns balance on/off, and redirects to
// menus 2 to 5 or either print data mode.
void HandleMenuKey(char input) {
  if (input == 'x') {
    StopAndCancel();
  } else if (input == '0') {
    balanceState = 0;
    appliedDuty = 0;
    FLYWHEEL_MOTOR.setDuty(0);
    Serial.println("Balance off");
  } else if (input == '1') {
    if (imuInitializationSucceeded == false) {
      Serial.println("Balance unavailable: IMU init failed; reset required");
    } else if (sensorFaultLatched || validSensorSample == false) {
      Serial.println("Balance unavailable: waiting for valid sensors");
    } else {
      balanceState = 1;
      if (tippedOver) {
        Serial.println("Balance on (waiting for upright)");
      } else {
        Serial.println("Balance on");
      }
    }
  } else if (input >= '2' && input <= '5') {
    stateMenu = input - '0';
    ResetValueInput();
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

// Read at most the shared character budget, even while changing menu states.
void ProcessSerialInput() {
  int inputCount = 0;
  while (Serial.available() > 0 && inputCount < MAX_INPUT_CHARS_PER_UPDATE) {
    char input = Serial.read();
    inputCount = inputCount + 1;

    if (discardingCanceledValue) {
      lastValueInputTime = millis();
      if (input == '\r' || input == '\n') {
        discardingCanceledValue = false;
      }
      continue;
    }

    if (input == '\r' || input == '\n') {
      if (stateMenu >= 2 && stateMenu <= 5) {
        SetValueFromInput();
      }
    } else if (stateMenu == -1 || stateMenu == -2) {
      stateMenu = 1;
      if (input == 'p' || input == 'm') {
        stateMenu = 0;
      } else {
        HandleMenuKey(input);
      }
    } else if (stateMenu == 0 || stateMenu == 1) {
      stateMenu = 1;
      HandleMenuKey(input);
    } else if (stateMenu >= 2 && stateMenu <= 5) {
      ReadValueCharacter(input);
    }
  }
}

void ReadValueCharacter(char input) {
  if (input == 'x') {
    StopAndCancel();
    discardingCanceledValue = true;
    lastValueInputTime = millis();
    return;
  }

  lastValueInputTime = millis();
  if (valueInputInvalid) {
    return;
  }

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
  } else {
    valueInputInvalid = true;
  }
}

void ResetValueInput() {
  valueInputLength = 0;
  valueInputHasPoint = false;
  valueInputHasDigit = false;
  valueInputInvalid = false;
}

void RejectValueInput() {
  ResetValueInput();
  stateMenu = 0;
  Serial.println("Invalid number; value unchanged");
}

void SetValueFromInput() {
  if (valueInputInvalid) {
    RejectValueInput();
    return;
  }

  if (valueInputLength == 0) {
    return;
  }

  // a sign or decimal point by itself is not a number.
  if (valueInputHasDigit == false) {
    RejectValueInput();
    return;
  }

  // '\0' marks the end of the text. atof converts that text to a number.
  valueInput[valueInputLength] = '\0';
  float enteredValue = atof(valueInput);
  ResetValueInput();

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

void StopAndCancel() {
  ResetValueInput();
  balanceState = 0;
  appliedDuty = 0;
  FLYWHEEL_MOTOR.setDuty(0);
  stateMenu = 1;
  Serial.println("Balance off");
}

void PrintValuePrompt() {
  if (stateMenu == 2) {
    Serial.println("Enter the desired set point (degrees; x stops):");
  } else if (stateMenu == 3) {
    Serial.println("Enter kp (x stops):");
  } else if (stateMenu == 4) {
    Serial.println("Enter kd (x stops):");
  } else if (stateMenu == 5) {
    Serial.println("Enter kf (x stops):");
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
  Serial.println("p data | e debug | v full data | m menu | x stop/cancel");
}

void PrintDebugMenu() {
  Serial.println("Extra debug:");
  Serial.println("v: full data (error, efforts, gains, balance)");
  Serial.println("p: basic data | m: main menu | 0: balance off | x: stop/cancel");
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

bool TriggerEvent(unsigned long t1, unsigned long t2, int desFreq) {
  // micros() counts microseconds, so one second is 1,000,000 counts.
  unsigned long timeBetweenEvents = 1000000UL / desFreq;
  if (t1 - t2 >= timeBetweenEvents) {
    return true;
  } else {
    return false;
  }
}
