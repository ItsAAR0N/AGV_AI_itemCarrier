#include <SoftwareSerial.h>

char val;
  
void setup() {
  //pinMode(15, INPUT_PULLUP);
  Serial.begin(115200);
  Serial3.begin(38400);
  Serial.println("Bluetooth Serial started at 9600 baud");
}

void loop() {
  
  Serial3.write('1');
    
  Serial3.flush();
  
  delay(10);
}
