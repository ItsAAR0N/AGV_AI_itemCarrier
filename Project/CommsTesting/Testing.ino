void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Check if the received command is "Detected"
    if (command == 'D') {
      // Serial.println("Object Detected!"); // Do something...
      Serial.println("A"); // Handshake protocol
    }
  }
}
