int onBoardLed = 13;

void setup()
{
  Serial.begin(115200);
  // Set pin 13 as an output
  pinMode(onBoardLed, OUTPUT);

  while (!Serial) {
    ; // Wait for serial port to connect.
  }
}

const char TERMINATOR = "|";

void loop()
{
  if (Serial.available() > 0)
  {
    String commandFromJetson = Serial.readStringUntil(TERMINATOR);
  
    // Confirm
    String ackMsg = "Hello Jetson! This is what I got: " + commandFromJetson; // String(messageBuffer);
    Serial.print(ackMsg);
  }
  blinkOnboardLED();
  // delay(500);
}

void blinkOnboardLED()
{
  // Turn the LED on
  digitalWrite(onBoardLed, HIGH);
  delay(1000); // Wait for 1 second

  // Turn the LED off
  digitalWrite(onBoardLed, LOW);
  delay(1000); // Wait for 1 second
}
