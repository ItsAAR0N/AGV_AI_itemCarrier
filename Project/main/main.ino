/* =================================================
Author(s): Aaron Shek, Li Ming Ip, ...
University of Hong Kong
Date of last edit: 15/04/24
AGV + Item/Message Carrier System (MAIN)
   =================================================
*/

// Required libraries are as follows:
// Adafruit_BusIO, Adafruit_GFX_Library, Adafruit_SSD1306, Servo-1.1.6, SoftwareSerial-Master
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <Servo.h>
// #include <MPU6050_light.h>
#include <string.h>

// Global definitions
// OLED Declaration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int oldV=1, newV=0;

// Servo motor defintions
int pan = 90; 
int tilt = 120;
int window_size = 0;
// int BT_alive_cnt = 0;
int voltCount = 0;
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
unsigned long time;
char incomingInstr;

// Tilt and Pan Servo declaration
#define PANPIN 48 // PL1
#define TILTPIN 47// PL2

// Obstacle Avoidance Servo declaration
int servoPin = 40; // PG0

// HC-SR04 Ultrasonic sensor declaration
#define ECHOPINA 25 // PA3 (pin 25)
#define TRIGPINA 28 // PA6 (pin 28)

#define TRIGPINB 33 // PC4 (pin 33)
#define ECHOPINB 32 // PC5 (pin 32)

// Ultrasonic sensor (obstacle avoidance) declaration
#define TRIGPINC 24 // PA2 (pin 24)
#define ECHOPINC 22 // PA0 (pin 22)

// IR sensor declaration
#define L_S A0 // Left IR sensor
#define R_S A2 // Right IR sensor

long durationR; long durationL; 
int distanceR; int distanceL; 
const int num_readings = 1; // Number of readings to average

int ultraDistance_L, ultraDistance_F, ultraDistance_R; 
int PresetDistance = 30;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 6   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);} while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);} while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);} while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);} while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);} while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);} while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);} while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);} while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);} while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);} while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);} while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);} while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

uint8_t Motor_PWM = 300;  // Slow

//    CAR MOVEMENTS
//    FORWARD  (BACK)
//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

void BACK()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    BACK (ADVANCE)
//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    LEFT_1
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM+15); 
  MOTORB_FORWARD(Motor_PWM+15);
  MOTORC_BACKOFF(Motor_PWM+ 15); 
  MOTORD_STOP(Motor_PWM+15);
}

//    RIGHT
//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    SW
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

// ######################

//
//    ↑B-----A↑
//     |  ↗  |
//     | ↗   |
//    ↑D-----C=
void BEAR_RIGHT_CUSTOM() {
  MOTORA_STOP(Motor_PWM); //10*1.3=13
  MOTORB_FORWARD(Motor_PWM+16);
  MOTORC_FORWARD(Motor_PWM+23); 
  MOTORD_STOP(Motor_PWM); //15*1.3=19.5
}

//    
//    ↑B-----A=
//     | ↙   |
//     |  ↙  |
//    =D-----C↑
void BEAR_LEFT_CUSTOM() {
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM+16);
  MOTORC_BACKOFF(Motor_PWM+23); 
  MOTORD_STOP(Motor_PWM);
}

//    RIGHT
//    ↓B-----A↑
//     |  ←  |
//     |  ←  |
//    ↑D-----C↓
void RIGHT_CUSTOMPWM_2()
{
  MOTORA_FORWARD(1900); // 17
  MOTORB_FORWARD(1900); // 16
  MOTORC_BACKOFF(1900); // -4
  MOTORD_BACKOFF(1900);
}

//    LEFT
//    ↑B-----A↓
//     |  →  |
//     |  →  |
//    ↓D-----C↑
void LEFT_CUSTOMPWM_2()
{
  MOTORA_BACKOFF(1900); // 11
  MOTORB_BACKOFF(1900); // 12
  MOTORC_FORWARD(1900); 
  MOTORD_FORWARD(1900); // 2
}

//    LEFT_1
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1_CUSTOM()
{
  MOTORA_STOP(Motor_PWM+16); 
  MOTORB_FORWARD(Motor_PWM+16);
  MOTORC_BACKOFF(Motor_PWM+16); 
  MOTORD_STOP(Motor_PWM+16);
}

//    RIGHT_1
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1_CUSTOM()
{
  MOTORA_BACKOFF(Motor_PWM+18); // Wheel installation is back to front (hence "backoff")
  MOTORB_STOP(Motor_PWM+18);
  MOTORC_STOP(Motor_PWM+18); 
  MOTORD_FORWARD(Motor_PWM+18);
}

// ######################

//    NE
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM+15); 
  MOTORB_STOP(Motor_PWM+15);
  MOTORC_STOP(Motor_PWM+15); 
  MOTORD_FORWARD(Motor_PWM+15);
}

//    left
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{ 
  Motor_PWM = 1900;
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    SE 
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ROTATE CLOCKWISE
//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ROTATE ANTI-CLOCKWISE
//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    STOP
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

// UART control interface -- For servo
void UART_Control()
{
  String myString;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }

    int commaIndex = myString.indexOf(','); // Split data in bracket (a, b, c)
    // Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
  }
}

// Display to OLED 2 Lines
void displayText(String text1, String text2, int cursorL1, int cursorL2, int cursorL3, int cursorL4) {
  display.clearDisplay();
  display.setCursor(cursorL1, cursorL2);     // Start at top-left corner
  display.println(text1);
  display.setCursor(cursorL3, cursorL4);     // Start at top-left corner
  display.println(text2);
  display.display();
}
// 3 lines
void displayText(String text1, String text2, String text3 , int cursorL1, int cursorL2, int cursorL3, int cursorL4, int cursorL5, int cursorL6) {
  display.clearDisplay();
  display.setCursor(cursorL1, cursorL2);     // Start at top-left corner
  display.println(text1);
  display.setCursor(cursorL3, cursorL4);     // Start at top-left corner
  display.println(text2);
  display.setCursor(cursorL5, cursorL6);     // Start at top-left corner
  display.println(text3);
  display.display();
}

// Voltage Readings transmitter (sends them via Serial3)
void sendVolt(){
  newV = analogRead(A0);
  if(newV!=oldV) {
    if (!Serial3.available()) {
      Serial3.println(newV);
      // Serial.println(newV);
    }
  }
  oldV=newV;
}

// Manual interface control (send car inst. via serial monitor)
void interface_control() { // Manual Serial interface 
  delay(10);
  if (Serial.available() > 0) { 
    // Read incoming data
    incomingInstr = Serial.read();
    incomingInstr = toupper(incomingInstr);
    // Serial.print("Received: ");
    // Serial.println(incomingInstr);
    delay(10); // Add a small delay after reading serial data
  }

  switch (incomingInstr)
  {
    case 'A': ADVANCE(); break;
    case 'B':  RIGHT_2(); break;
    case 'C':  rotate_1(); break;
    case 'D':  RIGHT_3(); break;
    case 'E':  BACK(); break;
    case 'F':  LEFT_3(); break;
    case 'G':  rotate_2(); break;
    case 'H':  LEFT_2(); break;
    case 'I':  STOP(); break;
    case 'J':  STOP(); break;
    case 'K':  LEFT_2(); break;
    case 'L':  RIGHT_2(); break;
    case 'M':  Motor_PWM = 1000; break; // Serial.print("PWM changed to: "); Serial.println(Motor_PWM); 
    case 'N':  Motor_PWM = 500; break;
  }
}

// Read HC-SR04 sensor (Ultrasonic Reading) for forward reading
void ultrasonic_reading_forward() {
  // Arrays to store readings for each sensor
  long durationsR[num_readings]; long durationsL[num_readings];
  // Sensoring (RIGHT)
  for (int i = 0; i < num_readings; i++) {
    digitalWrite(TRIGPINA, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPINA, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPINA, LOW);
    durationsR[i] = pulseIn(ECHOPINA, HIGH);
    // delay(50); // Delay between readings to avoid interference
  }
  delayMicroseconds(10);
  // Sensoring (LEFT)
  for (int i = 0; i < num_readings; i++) {
    digitalWrite(TRIGPINB, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGPINB, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPINB, LOW);
    durationsL[i] = pulseIn(ECHOPINB, HIGH);
    // delay(50); // Delay between readings to avoid interference
  }
  
  // Calculate average duration for right sensor
  long averageDurationR = 0;
  for (int i = 0; i < num_readings; i++) {
    averageDurationR += durationsR[i]; // Add up all of them
  }
  averageDurationR = averageDurationR/num_readings;
  distanceR = averageDurationR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
  // Calculate average duration for left sensor
  long averageDurationL = 0;
  for (int i = 0; i < num_readings; i++) {
    averageDurationL += durationsL[i]; // Add up all of them
  }
  averageDurationL = averageDurationL/num_readings;
  distanceL = averageDurationL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  
//  Serial.print("Distance R (U): ");
//  Serial.print(distanceR);
//  Serial.println(" cm");
//  Serial.print("Distance L(U): ");
//  Serial.print(distanceL);
//  Serial.println(" cm");
  String messageleft = "LEFT (U): ";
  messageleft += distanceL;
  String messageright = "RIGHT (U): ";
  messageright += distanceR;
  displayText(messageleft, messageright, 0, 0, 0, 10); 
}

long ultrasonic_obstacle_sensing() {
  digitalWrite(TRIGPINC, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPINC, HIGH);
  delayMicroseconds(10);
  long duration = pulseIn(ECHOPINC, HIGH);
  return duration * 0.034 / 2;
}

// Advance to wall until specified distance using Ultrasonic sensors
void advance_until_distance(int dis) {
  int target_distance;
  ultrasonic_reading_forward();
  if (dis == 5) {
    target_distance = dis + 4; // 2.5 = basic error
  } else {
    target_distance = dis + 5; // around 7-8 cm for this speed };
  };

  // Advance until the target distance is reached
  while (true) {
    ultrasonic_reading_forward(); // Update ultrasonic readings
    BACK();
    if (dis == 5){
      STOP();
      delay(100);
      BACK();
    }
    // Assuming distanceL and distanceR are continuously being updated
    if (distanceL < target_distance || distanceR < target_distance) {
      STOP();
      break;
    } 
  } 

  // Optionally display the final distances for verification
  String messageleft = "LEFT (U): " + String(distanceL);
  String messageright = "RIGHT (U): " + String(distanceR);
  displayText(messageleft, messageright, 0, 0, 0, 10); 
}

// Generate PWM signal to control servo
void servo_pulse(int servoPin, int angle) { // DONE ✔
  int PWM = (angle * 11) + 500; // Convert angle to ms
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(PWM);
  digitalWrite(servoPin, LOW);
  delay(50);
}

void distanceComparison_obstacle() { // NOT DONE ×
  if (ultraDistance_L > ultraDistance_R) {
    // Go left
    //RIGHT_CUSTOMPWM_2(); // Relative to car dir.
    LEFT_1_CUSTOM();
    delay(2000);
    BACK(); // Back is actually forwards in our case
    delay(500);
    while (digitalRead(R_S) == 0) {
      RIGHT_1_CUSTOM();
      delay(50);
    }
    STOP();
    delay(1000);
    // BACK();
    //delay(1500);
    //RIGHT_1();
    //delay(500);
  } else {
    //LEFT_CUSTOMPWM_2();
    RIGHT_1_CUSTOM();
    delay(2000);
    BACK();
    delay(500);
    while (digitalRead(L_S) == 0) {
      LEFT_1_CUSTOM();
      delay(50);
    }
    STOP();
    delay(1000);
    // BACK();
    // delay(1500);
    //LEFT_1();
    //delay(500);
  }
}

void CheckSides() { // DONE ✔
  STOP(); 
  delay(100);
  for (int angle = 70; angle <= 140; angle += 5) { // Rotate right
    servo_pulse(servoPin, angle); // Check right obstacles
  }
  delay(300);

  ultraDistance_R = ultrasonic_obstacle_sensing(); 
  displayText("Right Distance =", String(ultraDistance_R), 0, 0, 0, 10); // Display info on LCD
  delay(100);

  for (int angle = 140; angle >= 0; angle -= 5) { // Rotate left
    servo_pulse(servoPin, angle); // Check left obstacles
  }
  delay(500);

  ultraDistance_L = ultrasonic_obstacle_sensing();
  displayText("Left Distance =", String(ultraDistance_L), 0, 0, 0, 10);
  delay(100);

  for (int angle = 0; angle <= 70; angle += 5) { // Rotate back to forward pos
    servo_pulse(servoPin, angle);   
  }
  delay(300);
  distanceComparison_obstacle();
}

void lineFollower_obstacle_avoidance() { // DONE ✔
  ultraDistance_F = ultrasonic_obstacle_sensing();
  // Display to OLED
  String messageTop = "Forward D: " + String(ultraDistance_F);
  String messageMiddle = "Left: " + String((digitalRead(L_S) == 1 ? "Black" : "White"));
  String messageBottom = "Right: " + String((digitalRead(R_S) == 1 ? "Black" : "White"));
  displayText(messageTop, messageMiddle, messageBottom, 0, 0, 0, 10, 0, 20);
  // displayText("Forward Distance =", String(ultraDistance_F), 0, 0, 0, 10); // Display info on LCD

  // If Right Sensor and Left Sensor are at White color then it will call forward function
  if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0 )) {
    if (ultraDistance_F > PresetDistance) {
      BACK(); // Advance
      delay(100);
      STOP();
    }
    //} else if (((digitalRead(R_S) == 0) && (ultraDistance_F < PresetDistance)) || ((digitalRead(L_S) == 0) && (ultraDistance_F < PresetDistance))) {
    //  CheckSides();
    //} 
    else {
      CheckSides();// CheckSides(); // Obstcale detected, finding alternative path
    }
  } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) { 
    // If Right Sensor is Black and Left Sensor is White then it will call Right function
    BEAR_RIGHT_CUSTOM();
    delay(150);
    STOP();
  } else if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) {
    // STOP();
    // delay(50);
    // RIGHT_1(); // If Right Sensor is White and Left Sensor is Black then it will call Left function  
    BEAR_LEFT_CUSTOM();
    delay(150);
    STOP();
  } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) { 
    STOP(); // Destination reached
    delay(5000);
  }

  delay(10);
}

void JetsonCommunication() { // DONE ✔
  if (SERIAL.available() > 0) {
    // Read the incoming byte
    char command = Serial.read();

    // Check if the received command is "Detected"
    if (command == 'D') {
      // Serial.println("Object Detected!"); // Do something...
      Serial.println("A"); // Handshake protocol
    }
  }
}

void testCar() {
  uint8_t Motor_PWM = 300;
//  LEFT_1_CUSTOM();
//  delay(3000);
//  STOP();
//  delay(1000);
//  RIGHT_1_CUSTOM();
//  delay(3000);
//  STOP();
//  delay(1000);
  BEAR_LEFT_CUSTOM();
//  // delay(3000);
//  // STOP();
  //LEFT_1_CUSTOM();
}


void setup() {
  // Setup Serial interface
  SERIAL.begin(115200); // USB Serial setup
  // SERIAL.println("ROBOT START");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup

  // Setup Servo motors
  // Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);

  // OLED Setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  displayText("Ai Robot", "", 0, 0, 0, 0);

  // Setup HC-SR04 sensors
  pinMode(TRIGPINA, OUTPUT); 
  pinMode(ECHOPINA, INPUT);
  pinMode(TRIGPINB, OUTPUT); 
  pinMode(ECHOPINB, INPUT);
  pinMode(TRIGPINC, OUTPUT); 
  pinMode(ECHOPINC, INPUT);
    
  // Setup IR sensors
  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);

  // Setup Servo 
  pinMode(servoPin, OUTPUT);

  servo_pulse(servoPin, 0); // Set to zero angle
  // Re-align Servo
  for (int angle = 70; angle <= 140; angle += 5) {
    servo_pulse(servoPin, angle);
  }
  delay(1000);
  for (int angle = 140; angle >= 0; angle -= 5) {
    servo_pulse(servoPin, angle);
  }
  delay(1000);
  for (int angle = 0; angle <= 70; angle += 5) {
    servo_pulse(servoPin, angle);
  }
  ultraDistance_F = ultrasonic_obstacle_sensing();
  
  // Setup Voltage detector
  pinMode(A0, INPUT);

  displayText("TESTING", "10 Seconds...", 0, 0, 0, 10);
  delay(10000); // Delay 10 seconds
  display.clearDisplay();
  delay(500);
}

void loop() {
  // Run the code every 5 ms
  if (millis() > (time + 5)) {
    voltCount++; // Voltage r    while(true) {
//      testCar();
//    }eading
    time = millis();
    //IRTest();
    // UART_Control(); //get USB and BT serial data -- For servo camera control
    // interface_control(); // Manual control 
//
//    while(true) {
//      testCar();
//    }
    
    while (true) { // Run algorith continuously for testing
      JetsonCommunication(); 
      lineFollower_obstacle_avoidance();
    }
    
    
    // Constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);

    // Send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);

    // Voltage Reading
    if (voltCount>=5){
      voltCount=0;
      // sendVolt();
    }
  }
}
