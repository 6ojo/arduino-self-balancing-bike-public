// Declare Variables
int i = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Add delay code
  MyDelay();

  // Print instructions
  Serial.println("These instructions aren't always visibile because they are printed prior to opening the serial monitor");
  Serial.println("Enter a single character or number, then press enter!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    char byte = Serial.read();
    PrintByte(i, byte);
    i++;

    // put function here to clear the Serial Buffer
    ClearSerialBuffer(); // create this function below

  }
}

// Declare your function

void MyDelay() {
  while (!Serial){
    delay(10);
  }
}

void ClearSerialBuffer(){
  delay(100);
  while (Serial.available()>0){
    char trash = Serial.read();
  }
}

void PrintByte(int i, char c){
  Serial.print("Byte: "); Serial.print(i); Serial.print(" "); Serial.println(c);
}
