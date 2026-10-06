#include <Wire.h>
#include <ArduinoMotorCarrier.h>
#include <ArduinoBLE.h>
#include <stdlib.h>
#include "BNO055_support.h"

// Nordic UART Service UUIDs let a phone send menu commands and receive output.
BLEService wirelessService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic wirelessRx("6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
                              BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic wirelessTx("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                              BLERead | BLENotify, 20);

bool wirelessReady = false;
bool wirelessWasConnected = false;
char wirelessOutputBuffer[20];
int wirelessOutputLength = 0;

void FlushWirelessOutput();

class MirroredOutput : public Print {
 public:
  size_t write(uint8_t value) override {
    Serial.write(value);
    if (wirelessReady) {
      wirelessOutputBuffer[wirelessOutputLength] = value;
      wirelessOutputLength++;
      if (value == '\n' || wirelessOutputLength >= sizeof(wirelessOutputBuffer)) {
        FlushWirelessOutput();
      }
    }
    return 1;
  }
};

MirroredOutput Output;

void FlushWirelessOutput() {
  if (wirelessReady && wirelessOutputLength > 0 && wirelessTx.subscribed()) {
    wirelessTx.writeValue((const uint8_t *)wirelessOutputBuffer,
                          wirelessOutputLength);
  }
  wirelessOutputLength = 0;
}

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

  if (BLE.begin()) {
    BLE.setLocalName("MA6-Wireless");
    BLE.setAdvertisedService(wirelessService);
    wirelessService.addCharacteristic(wirelessRx);
    wirelessService.addCharacteristic(wirelessTx);
    BLE.addService(wirelessService);
    BLE.advertise();
    wirelessReady = true;
    Output.println("BLE ready: connect to MA6-Wireless with a BLE UART app");
  } else {
    Output.println("BLE startup failed; USB serial remains available");
  }

  // start I2C
  Wire.begin();

  if (!controller.begin()) {
    Output.println("Motor Carrier connection failed");
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
  if (wirelessReady) {
    BLE.poll();
    BLEDevice central = BLE.central();
    if (central) {
      wirelessWasConnected = true;
    } else if (wirelessWasConnected) {
      wirelessWasConnected = false;
      BLE.advertise();
    }
  }

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
    // MA6 defines error change as previous minus current error.
    errorChange = previousError - error;
    previousError = error;

    // A left lean needs positive flywheel effort to push the bike right.
    pEffort = -kp * error;
    // With previous-minus-current error, a growing left lean produces
    // positive D effort, also opposing the bike's falling motion.
    dEffort = kd * errorChange;
    // Flywheel damping restores the response used in the earlier MA6 runs.
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
        Output.println("Tip detected: braking flywheel");
      } else {
        Output.println("Upright: flywheel re-armed");
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

    if (wirelessReady && wirelessRx.written()) {
      uint8_t inputBuffer[20];
      int inputLength = wirelessRx.readValue(inputBuffer, sizeof(inputBuffer));
      for (int i = 0; i < inputLength; i++) {
        HandleMenuInput((char)inputBuffer[i]);
      }
    }

    while (Serial.available() > 0) {
      HandleMenuInput((char)Serial.read());
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
    Output.println("Balance off");
  } else if (input == '1') {
    balanceState = 1;
    if (tippedOver) {
      Output.println("Balance on (waiting for upright)");
    } else {
      Output.println("Balance on");
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
    Output.println("Input parser error");
  }
}

// Route USB serial and BLE UART characters through the same menu parser.
void HandleMenuInput(char input) {
  if (stateMenu == 1) {
    HandleMenuKey(input);
  } else if (stateMenu >= 2 && stateMenu <= 5) {
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
  } else if (stateMenu == -1 || stateMenu == -2) {
    // Ignore line endings so a phone can send newline-terminated commands.
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

void RejectValueInput() {
  valueInputLength = 0;
  valueInputHasPoint = false;
  valueInputHasDigit = false;
  ClearSerialBuffer();
  Output.println("Invalid number; value unchanged");
  PrintValuePrompt();
}

void SetValueFromInput() {
  // a sign or decimal point by itself is not a number.
  if (valueInputHasDigit == false) {
    valueInputLength = 0;
    valueInputHasPoint = false;
    valueInputHasDigit = false;
    Output.println("Invalid number; value unchanged");
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
    Output.print("Set point set to ");
  } else if (stateMenu == 3) {
    kp = enteredValue;
    Output.print("kp set to ");
  } else if (stateMenu == 4) {
    kd = enteredValue;
    Output.print("kd set to ");
  } else if (stateMenu == 5) {
    kf = enteredValue;
    Output.print("kf set to ");
  }
  Output.println(enteredValue, 4);
  stateMenu = 1;
}

void PrintValuePrompt() {
  if (stateMenu == 2) {
    Output.println("Enter the desired set point (degrees):");
  } else if (stateMenu == 3) {
    Output.println("Enter kp:");
  } else if (stateMenu == 4) {
    Output.println("Enter kd:");
  } else if (stateMenu == 5) {
    Output.println("Enter kf:");
  }
}

void PrintMenu() {
  Output.print("Balance: ");
  if (balanceState == 1) {
    Output.print("on");
  } else {
    Output.print("off");
  }
  if (tippedOver) {
    Output.println(" (tipped over)");
  } else {
    Output.println();
  }
  Output.print("Set point: ");
  Output.print(setPoint, 2);
  Output.print("  kp: ");
  Output.print(kp, 4);
  Output.print("  kd: ");
  Output.print(kd, 4);
  Output.print("  kf: ");
  Output.println(kf, 4);
  Output.println("0 off | 1 on | 2 set point");
  Output.println("3 kp | 4 kd | 5 kf");
  Output.println("p basic data | e extra debug | m menu");
}

void PrintDebugMenu() {
  Output.println("Extra debug:");
  Output.println("v: full data (error, efforts, gains, balance)");
  Output.println("p: basic data | m: main menu | 0: balance off");
}

void PrintDataHeader() {
  Output.println("roll\tsetPt\tflySpd\tduty\ttip");
}

void PrintData() {
  Output.print(roll, 2);
  Output.print('\t');
  Output.print(setPoint, 2);
  Output.print('\t');
  Output.print(flywheelSpeed);
  Output.print('\t');
  Output.print(appliedDuty);
  Output.print('\t');
  if (tippedOver) {
    Output.println(1);
  } else {
    Output.println(0);
  }
}

void PrintDebugDataHeader() {
  Output.println("roll\tsetPt\terror\tflySpd\tpEff\tdEff\tfEff\t"
                 "effort\tduty\tbal\ttip\tkp\tkd\tkf");
}

void PrintDebugData() {
  Output.print(roll, 2);
  Output.print('\t');
  Output.print(setPoint, 2);
  Output.print('\t');
  Output.print(error, 2);
  Output.print('\t');
  Output.print(flywheelSpeed);
  Output.print('\t');
  Output.print(pEffort, 2);
  Output.print('\t');
  Output.print(dEffort, 2);
  Output.print('\t');
  Output.print(fEffort, 2);
  Output.print('\t');
  Output.print(requestedEffort, 2);
  Output.print('\t');
  Output.print(appliedDuty);
  Output.print('\t');
  Output.print(balanceState);
  Output.print('\t');
  if (tippedOver) {
    Output.print(1);
  } else {
    Output.print(0);
  }
  Output.print('\t');
  Output.print(kp, 4);
  Output.print('\t');
  Output.print(kd, 4);
  Output.print('\t');
  Output.println(kf, 4);
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
