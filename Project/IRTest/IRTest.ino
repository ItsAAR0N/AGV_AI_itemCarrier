// Program to calibrate and test the IR sensors
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

const int sensorPin_R = A2; // Analogue 2 pin
const int sensorPin_L = A0; // Analogue 0 pin 

int sensorValue_R = 0;
int sensorValue_L = 0;

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sensorPin_R, INPUT);
  pinMode(sensorPin_L, INPUT);

  // OLED Setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // displayText("Ai Robot", "", 0, 0, 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue_R = digitalRead(sensorPin_R);
  Serial.println("=========");
  Serial.println("Right: ");
  if (sensorValue_R == HIGH) {
    Serial.println("Black");
  } else {
    Serial.println("White");
  }
  sensorValue_L = digitalRead(sensorPin_L);
  Serial.println("Left: ");
  if (sensorValue_L == HIGH) {
    Serial.println("Black");
  } else {
    Serial.println("White");
  }

  // Display to OLED
  String messageMiddle = "Left: " + String((digitalRead(sensorPin_L) == 1 ? "Black" : "White"));
  String messageBottom = "Right: " + String((digitalRead(sensorPin_R) == 1 ? "Black" : "White"));
  displayText("", messageMiddle, messageBottom, 0, 0, 0, 10, 0, 20);
  
  delay(100);
}
