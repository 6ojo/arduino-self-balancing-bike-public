// Define Macros
#define LED_PIN (11)
#define PERIOD_100HZ (10)
#define PERIOD_10HZ (100)

// Declare State variables (for LED action
// and menu action)
bool LEDState = false;
char menuState = 'p';

unsigned long prevTimeA = 0;
unsigned long prevTimeB = 0;

int dutyCycle = 100;
int dutyScaled = 255;
int LEDPeriod = 500;
void setup() {
  // put your setup code here, to run once:
  // Initialize Serial Monitor
  Serial.begin(9600);
  // set up on board LED pin
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currTime = millis();
  // 100hz wrrapper
  if ((currTime - prevTimeA) >= PERIOD_100HZ) {
    prevTimeA = currTime;

    // Check LED state (part A)
    if (LEDState && (currTime % LEDPeriod) < (LEDPeriod / 2)) {
      analogWrite(LED_PIN, dutyScaled);
    } else {
      analogWrite(LED_PIN, 0);
    }
  }
  // 10hz wrapper
  if ((currTime - prevTimeB) >= PERIOD_10HZ) {
    prevTimeB = currTime;

    // Check menu state (part B)
    if (menuState == 'p') {
      // Print menu
      PrintMenu();


      // Set menu state to menu 1;
      menuState = '1';
    }

    // Check data entered on Serial monitor
    if (Serial.available() > 0) {
      // Check if Menu 1 (the main menu) (Part C)
      if (menuState == '1') {
        // read input as a byte (also known as a char)
        char input = Serial.read();

        // Clear serial buffer
        ClearSerialBuffer();

        // check input
        if (input == '0') { // is input '0'?
          // set LED state to Off
          LEDState = false;
        } else if (input == '1') { // is input '1'?
          // set LED state to on
          LEDState = true;
        } else if (input == 'm') { // is input 'm'?
          // Set menu to print mode
          menuState = 'p';

        } else if (input == 'd') { // is input 'd'?
          // set state menu to menu 2
          menuState = '2';
          // print instructions
          Serial.println("Please enter the LED Duty Cycle (0 to 100)");

        } else { // else case, useful for debugging.
          Serial.println("input parser error");
        }
      } else if (menuState == '2') { // Check if menu 2 (the second menu) (Part D)
        // read input as int
        dutyCycle = Serial.parseInt();
        dutyCycle = constrain(dutyCycle, 0, 100);
        dutyScaled = map(dutyCycle, 0, 100, 0, 255);
        Serial.println("Duty cycle set to:");
        Serial.println(dutyCycle);
        PrintMenu();

        // clear serial buffer
        ClearSerialBuffer();

        // If delay time is negative, set delay time to 1000 ms
        // what does this do
        // delay never changes?
         if (LEDPeriod < 0 ){
          LEDPeriod = 2000;
         }

        // set menu state to 1
        menuState = '1';
      }
    }
  }
}

// Define PrintMenu Function
void PrintMenu(){
  Serial.println("Duty cycle is:");
  Serial.println(dutyCycle);
  Serial.println("Scaled Duty cycle is:");
  Serial.println(dutyScaled);
  Serial.println("Choose an option:");
  Serial.println("0 to turn LED off");
  Serial.println("1 to turn LED on");
  Serial.println("m to print menu");
  Serial.println("d to set duty cycle");
}

// Define ClearSerialBuffer Function
void ClearSerialBuffer(){
  while (Serial.available() > 0) {
    Serial.read();
  }
}
