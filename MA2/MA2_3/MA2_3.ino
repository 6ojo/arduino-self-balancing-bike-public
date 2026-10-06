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
  if (TriggerEvent(currTime, prevTime1, 10)){
    // Calculate and print the frequency of the loop
    float loop1Freq = 1000000.0 / (currTime - prevTime1);

    Serial.print("The frequency of the 10Hz loop is ");
    Serial.println(loop1Freq, 2);
    // save loop 1 execution time as current time
    prevTime1 = currTime;
  }

  // Conditional for loop 2, a 1 HZ loop (PART B)
  if (TriggerEvent(currTime, prevTime2, 1)) {
    // Calculate and print the frequency of the loop
    float loop2Freq = 1000000.0 / (currTime - prevTime2);

    Serial.print("The frequency of the 1Hz loop is ");
    Serial.println(loop2Freq, 2);
    // save loop 2 execution time as current time
    prevTime2 = currTime;
  }

}


bool TriggerEvent(unsigned long currTime, unsigned long prevTime, int desFreq){
  unsigned long desPeriod = 1000000 / desFreq;
  unsigned long elapsedTime = currTime - prevTime;

  if (elapsedTime >= desPeriod) {
    return true;
  }
  else {
    return false;
  }
}
