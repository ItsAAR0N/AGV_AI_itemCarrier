/* =================================================
Author(s): Aaron Shek, Li Ming Ip
University of Hong Kong
Date of last edit: 21/03/24 
AGV + Item/Message Carrier System
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
#include <MPU6050_light.h>
#include <string.h>

// Global definitions
// OLED Declaration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;
unsigned long time;
char incomingInstr;

// Variables for light intensity to ADC reading equations
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc2, int_adc2_m, int_adc2_c;     
int int_left, int_right;



// MPU6050 Declaration
MPU6050 mpu(Wire);
unsigned longtimer = 0;

// HC-SR04 Declaration
#define ECHOPINA 25 // PA3 (pin 25)
#define TRIGPINA 28 // PA6 (pin 28)

#define TRIGPINB 33 // PA4 (pin 33)
#define ECHOPINB 32 // PA5 (pin 32)

long durationR; long durationL; 
int distanceR; int distanceL; 
const int num_readings = 1; // Number of readings to average
int ultracount = 0;

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

uint8_t Motor_PWM = 300;

// Car movements
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
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

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
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
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

// UART control interface
void UART_Control()
{
  String myString;
  char BT_Data = 0;
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
  //BT Control
  // Receive data from app and translate it to motor movements
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}

// Display to OLED
void displayText(String text1, String text2, int cursorL1, int cursorL2, int cursorL3, int cursorL4) {
  display.clearDisplay();
  display.setCursor(cursorL1, cursorL2);     // Start at top-left corner
  display.println(text1);
  display.setCursor(cursorL3, cursorL4);     // Start at top-left corner
  display.println(text2);
  display.display();
}

void displayText(String text1, String text2, String text3 , int cursorL1, int cursorL2, int cursorL3, int cursorL4, int cursorL5, int cursorL6) {
  display.clearDisplay();
  display.setCursor(cursorL1, cursorL2);     // Start at top-left corner
  display.println(text1);
  display.setCursor(cursorL3, cursorL4);     // Start at top-left corner
  display.println(text2);
  display.setCursor(cursorL4, cursorL5);     // Start at top-left corner
  display.println(text3);
  display.display();
}

// Voltage Readings transmitter (sends them via Serial3)
void sendVolt(){
  newV = analogRead(A0);
  if(newV!=oldV) {
    if (!Serial3.available()) {
      Serial3.println(newV);
      Serial.println(newV);
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
    Serial.print("Received: ");
    Serial.println(incomingInstr);
    delay(10); // Add a small delay after reading serial data
  }

  switch (incomingInstr)
  {
    case 'A': ADVANCE(); break;
    case 'B':  RIGHT_2(); break;
    case 'C':  rotate_1(); break;
    case 'D':  RIGHT_3(); break;
    case 'E':  BACK(1000, 1000, 1000, 1000); break;
    case 'F':  LEFT_3(); break;
    case 'G':  rotate_2(); break;
    case 'H':  LEFT_2(); break;
    case 'I':  STOP(); break;
    case 'J':  STOP(); break;
    case 'K':  LEFT_2(); break;
    case 'L':  RIGHT_2(); break;
    case 'M':  Motor_PWM = 1500; break; // Serial.print("PWM changed to: "); Serial.println(Motor_PWM); 
    case 'N':  Motor_PWM = 500; break;
  }
}

void Light_reading() {
  // Calculate the light intensity of the sensors in the range of [0, 100]
  int_left=(analogRead(A0)*0.9-int_adc0_c)/int_adc0_m;
  int_right=(analogRead(A2)-int_adc2_c)/int_adc2_m;  
  String messageleft = "LEFT (L): ";
  messageleft += int_left;
  String messageright = "RIGHT (L): ";
  messageright += int_right;
  displayText(messageleft, messageright, 0, 0, 0, 10); 
}

// Read IMU (Gyroscope Reading)
void IMU_reading() {
  mpu.update();
  Serial.print("Pitch (Angle_X) : ");
  Serial.print(mpu.getAngleX());
  Serial.print("  Roll  (Angle_Y) : ");
  Serial.print(mpu.getAngleY());
  Serial.print("  Yaw   (Angle_Z) : ");
  Serial.println(mpu.getAngleZ());
}

// Read HC-SR04 sensor (Ultrasonic Reading)
void ultrasonic_reading() {
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
  
  Serial.print("Distance R (U): ");
  Serial.print(distanceR);
  Serial.println(" cm");
  Serial.print("Distance L(U): ");
  Serial.print(distanceL);
  Serial.println(" cm");
  String messageleft = "LEFT (U): ";
  messageleft += distanceL;
  String messageright = "RIGHT (U): ";
  messageright += distanceR;
  displayText(messageleft, messageright, 0, 0, 0, 10); 
}

void align_with_wall(){
  const int tolerance = 1; // Tolerance in cm for alignment
  ultrasonic_reading();
  // Align the vehicle to be perpendicular to the wall
  while (abs(distanceL - distanceR) > tolerance) {
    if (distanceL > distanceR) {
      // If the left side is farther from the wall, rotate clockwise
      rotate_2(); // Continuously rotate clockwise
    } else if (distanceR > distanceL) {
      // If the right side is farther, rotate counterclockwise
      rotate_1(); // Continuously rotate counterclockwise
    }
    ultrasonic_reading();
    delay(10); // Short delay to stabilize sensor readings after rotation
  }
  STOP();
}

void advance_until_distance(int dis) {
  int target_distance;
  ultrasonic_reading();
  if (dis == 5) {
    target_distance = dis + 4; // 2.5 = basic error
  } else {
    target_distance = dis + 5; // around 7-8 cm for this speed };
  };

  // Advance until the target distance is reached
  while (true) {
    ultrasonic_reading(); // Update ultrasonic readings
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

void align_with_light_source() {
  const int alignment_tolerance = 10; // Adjust as needed
  // Read initial brightness values of the LDRs
  Light_reading();
  
  // Continue alignment process until brightness values are approximately equal
  while (abs(int_left - int_right) > alignment_tolerance) {
    
    // Determine the direction in which to move the car
    if (int_left > int_right) {
      // Move the car left
      LEFT_2(); // MOVE TO THE LEFT
      delay(180); // Delay for stabilization, adjust as needed
      STOP(); // Stop the car
    } 
    else if (int_left < int_right) {
      // Move the car right
      RIGHT_2(); // MOVE TO THE RIGHT
      delay(180); // Delay for stabilization, adjust as needed
      STOP(); // Stop the car
    }
    
    // Read brightness values again
    Light_reading();
    
    // Add a short delay to stabilize readings and avoid rapid changes
    delay(100); // Adjust delay as needed
  }
  // advance_until_distance(5);
  // Stop the car once alignment is achieved
  STOP();  
}

// Funtion to rotate CW
void rotate_CW(int degree) {
  // get initial angle
  int target_angle = mpu.getAngleZ() - degree + 15; // Stop 15 degree earlier to cancel momentum
  int temp_angle; 
  while (true){
    IMU_reading();
    temp_angle = mpu.getAngleZ();
    if (temp_angle <= target_angle){ 
      STOP();
      break;
    } else {
      // Rotate clockwise
      rotate_2(); // Call CW rotate function
      delay(100); // Delay between movements to avoid rapid changes
    }
  }
}

// Funtion to rotate CCW
void rotate_CCW(int degree) {
  // get initial angle
  int target_angle = mpu.getAngleZ() + degree - 25;
  int temp_angle; 
  Serial.println(target_angle);
  while (true){
    temp_angle = mpu.getAngleZ();
    IMU_reading();
    if (temp_angle >= target_angle){ 
      STOP();
      break;
    } else {
      // Rotate clockwise
      rotate_1(); // Call CCW rotate function
      delay(100); // Delay between movements to avoid rapid changes
    }
  }
}

// Program start
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("PROGRAM START");
  SERIAL.println("ROBOT CALIBRATION IN PROGRESS...");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  
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

  // LDR Setup
  // Measure the sensors reading at ambient light intensity
  displayText("Calibrating Light", "5 Seconds", 0, 0, 0, 10); 
  // Serial.println("Calibration in progress, put the sensors under the light (~ 10 sec) ......");
  // Serial.println("***********************");    
  delay(5000);
  // Variables to store readings
  int numReadings = 10; // Number of readings to average
  int total_adc0 = 0;
  int total_adc2 = 0;

  // Take multiple readings from each LDR and sum them up
  for (int i = 0; i < numReadings; i++) {
    total_adc0 += analogRead(A0);
    total_adc2 += analogRead(A2);
    delay(10); // Adjust delay as needed for stability
  }

  // Calculate the average readings
  int_adc0 = (total_adc0 *0.9) / numReadings; // avg 1
  int_adc2 = total_adc2 / numReadings; // avg 2

  // if (int_adc2 > int_adc0) { 
  //   int_adc2 = total_adc2 / numReadings - 30;
  // } else if (int_adc0 > int_adc2){
  //   int_adc2 = total_adc2 / numReadings;
  // }
  
  
  // Serial.print("Left: ");
  // Serial.println(int_adc0);
  // Serial.print("Right: ");
  // Serial.println(int_adc1)
  String messageleft = "LEFT (L): ";
  messageleft += int_adc0;
  String messageright = "RIGHT (L): ";
  messageright += int_adc2;
  displayText(messageleft, messageright, 0, 0, 0, 10); 

  delay(3000); 

  // Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
  // Serial.println("************ Put Fingers *****************");
  displayText("Calibration in progess", "Cover sensor with finger", 0, 0, 0, 10);
  delay(5000);        // delay 5000 ms
  // Serial.println("********* START Calibration **************");

  // measure the sensors reading at zero light intensity  
  int_adc0_c=analogRead(A0);   // Left sensor at zero light intensity
  int_adc2_c=analogRead(A2);   // Right sensor at zero light intensity

  // calculate the slope of light intensity to ADC reading equations  
  int_adc0_m=(int_adc0-int_adc0_c)/100;
  int_adc2_m=(int_adc2-int_adc2_c)/100;
  // delay(10000);        // delay 10000 ms 
   
  // Serial.println("\n******** Completed! Remove your hands ********");
  displayText("Completed!", "Remove finger", 0, 0, 0, 10);

  delay(2000);        // delay 2000 ms

  // IMU Setup
  Wire.begin();
  mpu.begin();
  // Serial.println("Calculating gyro offset, do not move MPU6050");
  displayText("Calculating Gyro offset", "Do not move MPU", 0, 0, 0, 10);
  delay(1000);
  mpu.calcGyroOffsets();                          // This does the calibration
  displayText("Robot calibrated", "All done...", 0, 0, 0, 10);

  // Setup HC-SR04 sensors
  pinMode(TRIGPINA, OUTPUT); 
  pinMode(ECHOPINA, INPUT);
  pinMode(TRIGPINB, OUTPUT); 
  pinMode(ECHOPINB, INPUT);

  // Setup Voltage detector
  pinMode(A0, INPUT);
}

// To be run continuously
void loop()
{
  // run the code in every 20ms
  if (millis() > (time + 5)) {
    voltCount++;
    ultracount++;
    time = millis();

    UART_Control(); //get USB and BT serial data
    interface_control();
    
    // Constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
    
    // READINGS
    // Light_reading();
    // IMU_reading();
    // ultrasonic_reading();

    delay(7000);
    align_with_wall();
    delay(2000);
    advance_until_distance(30);
    delay(2000);
    rotate_CW(90);
    delay(2000);
    rotate_CCW(270);
    delay(2000);
    rotate_CW(180);
    delay(2000);  
    ultrasonic_reading(); 
    String messageleft = "Distance: ";
    messageleft += "23";
    String messageright = "Angle: ";
    messageright += mpu.getAngleZ();
    displayText(messageleft, messageright, 0, 0, 0, 10); 
    delay(2000);
    align_with_light_source();
    delay(2000);
    advance_until_distance(5);
    delay(2000);
    while (true){
      STOP();
    }

    // Send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }
  if (voltCount>=5){
    voltCount=0;
    sendVolt();
  }
  BACK();
  // if (ultracount >= 6 ){
  //   ultracount=0;
  //   BACK(1000,1000,1000,1000);
  //   // sendVolt();
  // }
}
