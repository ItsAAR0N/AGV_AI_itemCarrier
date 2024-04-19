// Program to calibrate and test the IR sensors

const int sensorPin_R = A2; // PA3 (pin 25)
const int sensorPin_L = A0; // PA6 (pin 28)

int sensorValue_R = 0;
int sensorValue_L = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sensorPin_R, INPUT);
  pinMode(sensorPin_L, INPUT);
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
  delay(100);
}
