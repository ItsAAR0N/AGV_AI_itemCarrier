// Program to establish and test advanced communication between two interfaces (i.e., sending out commands)

int onBoardLed = 13;

void setup()
{
  Serial.begin(115200);
  pinMode(onBoardLed, OUTPUT);

  while (!Serial) {
    ; // Wait for serial port to connect.
  }
}

const char TERMINATOR = '|';

void loop()
{
  if (Serial.available() > 0)
  {
    String commandFromJetson = Serial.readStringUntil(TERMINATOR);
    executeCommand(commandFromJetson);
  }
  blinkOnboardLED();
}

void executeCommand(String command)
{
  if (command == "A")
  {
    Serial.println("Command A received and executed.");
    // Action for A..
  }
  else if (command == "B")
  {
    Serial.println("Command B received");
    // Continue..
  }
  else if (command == "C")
  {
    Serial.println("Command C received");
    // Continue..
  }
  else
  {
    Serial.println("INVALID COMMAND");
  }
}

void blinkOnboardLED()
{
  digitalWrite(onBoardLed, HIGH);
  delay(1000); // Wait for 1 second
  digitalWrite(onBoardLed, LOW);
  delay(1000); // Wait for 1 second
}
