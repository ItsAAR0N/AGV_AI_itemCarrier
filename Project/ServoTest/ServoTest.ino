// Code to test SG90 servo motor

#include <Servo.h>

const int servoPin = 40; // Pin for servo motor
// Servo servo;  // Create a servo object 

void setup() {
  // servo.attach(servoPin);  // Attach the servo to the specified pin
  pinMode(servoPin, OUTPUT);
}

// Generate PWM signal to control servo
void servo_pulse(int servoPin, int angle) {
  int PWM = (angle * 11) + 500; // Convert angle to ms
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(PWM);
  digitalWrite(servoPin, LOW);
  delay(50);
}

void loop() {
  // Re-align Servo
  // servo.write(0);
  servo_pulse(servoPin, 0);
  delay(5000);
  for (int angle = 70; angle <= 140; angle += 5) {
    servo_pulse(servoPin, angle);
  }
  delay(5000);
  for (int angle = 140; angle >= 0; angle -= 5) {
    servo_pulse(servoPin, angle);

  }
  delay(5000);
  for (int angle = 0; angle <= 70; angle += 5) {
    servo_pulse(servoPin, angle);
  }

//  delay(2000);
//  for (int angle = 140; angle >= 0; angle -= 5) {
//    servo.write(angle);
//    delay(50);
//  }
//  delay(2000);
//  for (int angle = 0; angle <= 70; angle += 5) {
//    servo.write(angle);
//    delay(50);
//  }


  // servo.write(180);
//  for (int angle = 0; angle <= 180; angle++) { // Sweep from 0 to 180 degrees
//    servo.write(angle); // Set the servo angle
//    delay(15); // Delay for smoother movement
//  }
//  
//  delay(500); // Pause at the end position
//  
//  for (int angle = 180; angle >= 0; angle--) { // Sweep from 180 to 0 degrees
//    servo.write(angle); // Set the servo angle
//    delay(15); // Delay for smoother movement
//  }
//  
  delay(10000); // Pause at the start position
}
