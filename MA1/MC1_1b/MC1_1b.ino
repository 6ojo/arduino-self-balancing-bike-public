// Initialize three global variables, a char, a
// short, and an int as zero
char varOne = 0;
short varTwo = 0;
int varThree = 0;

void setup() {
  // put your setup code here, to run once:

  // Initialize the Serial Monitor
  Serial.begin(9600);

}

void loop() {

  // Print the three variables in the order char, short, int.
  // The variable should be printed as numeric (not character
  // values). Review the "Format" for printing numbers
  // This should be followed by a new line
  Serial.print(varOne, DEC);
  Serial.print(" ");
  Serial.print(varTwo);
  Serial.print(" ");
  Serial.println(varThree);
  // increment the three variables
  varOne++;
  varTwo++;
  varThree++;
}
