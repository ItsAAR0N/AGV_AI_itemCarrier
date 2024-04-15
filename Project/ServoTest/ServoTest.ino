#include <Servo.h>

const int servoPin = 40; // Pin for servo motor
Servo servo;  // Create a servo object

void setup() {
  servo.attach(servoPin);  // Attach the servo to the specified pin
}

void loop() {
  for (int angle = 0; angle <= 180; angle++) { // Sweep from 0 to 180 degrees
    servo.write(angle); // Set the servo angle
    delay(15); // Delay for smoother movement
  }
  
  delay(500); // Pause at the end position
  
  for (int angle = 180; angle >= 0; angle--) { // Sweep from 180 to 0 degrees
    servo.write(angle); // Set the servo angle
    delay(15); // Delay for smoother movement
  }
  
  delay(500); // Pause at the start position
}
