// Include a macro for PI
#define PI_VALUE 3.14159

// include a global variable to store the
// previous loop time. Initialize to 0
float lastLoop = 0;


void setup() {
  // put your setup code here, to run once:

  // Initialize serial monitor
  Serial.begin(9600);

  Serial.print("t (sec)");
  Serial.print("\t");
  Serial.print("f(t)");
  Serial.print("\t");
  Serial.print("g(t)");
  Serial.print("\t");
  Serial.println("h(t)");
}

void loop() {
  // put your main code here, to run repeatedly:
  // get the current time in milliseconds. Save
  // as a floating point number
  float currTimeMils = millis();


  // convert time in milliseconds to seconds
  float currTimeSecs = currTimeMils / 1000;


  // calcualte f
  float fOfT = currTimeSecs * sin(2 * PI_VALUE * currTimeSecs);

  // calculate g
  float gOfT = pow(currTimeSecs, 2);

  // calculate h
  float hOfT = 1 / (currTimeSecs - lastLoop);

  // store loop time for next loop iteration
  lastLoop = currTimeSecs;


  // print data using tab format
  Serial.print(currTimeSecs);
  Serial.print("\t");
  Serial.print(fOfT);
  Serial.print("\t");
  Serial.print(gOfT);
  Serial.print("\t");
  Serial.println(hOfT);


  // delay 100 milliseconds
  delay(100);
}
