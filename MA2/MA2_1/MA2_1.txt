// Declare constants for loop period
const unsigned long LOOP1_PERIOD = 100000;
const unsigned long LOOP2_PERIOD = 1000000;


// Declare variables to save the last time each
// timed statement exectutes
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read current time in micros
  unsigned long currTime = micros();

  // Conditional for loop 1, a 10 HZ loop (Part A)
  if ((currTime - prevTime1) >= LOOP1_PERIOD){
    // Calculate and print the frequency of the loop
    float loop1Freq = 1000000.0 / (currTime - prevTime1);

    Serial.print("The frequency of the 10Hz loop is ");
    Serial.println(loop1Freq, 2);
    // save loop 1 execution time as current time
    prevTime1 = currTime;
  }

  // Conditional for loop 2, a 1 HZ loop (PART B)
  if ((currTime - prevTime2) >= LOOP2_PERIOD) {
    // Calculate and print the frequency of the loop
    float loop2Freq = 1000000.0 / (currTime - prevTime2);

    Serial.print("The frequency of the 1Hz loop is ");
    Serial.println(loop2Freq, 2);
    // save loop 2 execution time as current time
    prevTime2 = currTime;
  }
}
