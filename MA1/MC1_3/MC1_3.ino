// Add a Macro for pi
#define PI_VALUE 3.14159

void setup()
{
  // delay at the start to open the serial monitor in the IDE
  delay(5000);

  // Initialize the Serial Monitor
  Serial.begin(9600);

  // wait 10 secs before skipping a read data step;
  Serial.setTimeout(10000);

  // Query for desired angle, read as float
  Serial.println("What is your angle in degrees?");
  float inputDeg = Serial.parseFloat();
  Serial.print("You said: ");
  Serial.print(inputDeg);
  Serial.println(" degrees");

  // "Read" the sensor as a float. Use random() function
  float sensorPercent = (float)random(0, 100);
  Serial.print("The sensor is at: ");
  Serial.print(sensorPercent);
  Serial.println("%");

  // convert read angle to degrees
  float sensorDeg = sensorPercent * 360.0 / 100.0;

  // calculate error
  float errorDeg = inputDeg - sensorDeg;
  Serial.print("The error in degrees is: ");
  Serial.println(errorDeg);

  // convert to rad as int and float
  float errorRadFloat = errorDeg * PI_VALUE / 180.0;
  int errorRadInt = (int)errorRadFloat;

  Serial.print("The error in radians as an integer is: ");
  Serial.println(errorRadInt);
  Serial.print("The error in radians as a floating-point number is: ");
  Serial.println(errorRadFloat);
}

void loop()
{

}
