void setup() {
  // Start the hardware serial port for the serial monitor
  Serial.begin(9600);
  // Start the software serial port for the Bluetooth connection
  // Serial.begin(9600);
  Serial.println("Bluetooth Serial started at 9600 baud");
}
void loop() {
  // Check if data has been received from the Bluetooth connection
  //if (Serial3.available()) {
    // Serial.println("Available");
    char receivedChar = Serial.read();
    Serial.print("Received: ");
    Serial.println(receivedChar);
  //}
}
