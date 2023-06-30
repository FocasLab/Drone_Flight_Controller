#include <ESP32Servo.h>

#define MOTOR1_PIN 33
#define MOTOR2_PIN 25
#define MOTOR3_PIN 32
#define MOTOR4_PIN 26

unsigned long loop_timer;

Servo MOTOR1, MOTOR2, MOTOR3, MOTOR4;

void setup() {

  Serial.begin(115200);

  MOTOR1.attach(MOTOR1_PIN, 1000, 2000);
  MOTOR2.attach(MOTOR2_PIN, 1000, 2000);
  MOTOR3.attach(MOTOR3_PIN, 1000, 2000);
  MOTOR4.attach(MOTOR4_PIN, 1000, 2000);

  delay(1000);

  loop_timer = micros();
  while (micros() - loop_timer < 5000000) {
    MOTOR1.writeMicroseconds(1000);
    MOTOR2.writeMicroseconds(1000);
    MOTOR3.writeMicroseconds(1000);
    MOTOR4.writeMicroseconds(1000);
    //delay(4);
  }
  delay(100);
  loop_timer = micros();
  while (micros() - loop_timer < 5000000) {
    MOTOR1.writeMicroseconds(1000);
    MOTOR2.writeMicroseconds(1000);
    MOTOR3.writeMicroseconds(1000);
    MOTOR4.writeMicroseconds(1000);
    
    //delay(4);
  }
  delay(100);
  loop_timer = micros();
  
}

void loop() {

  MOTOR1.writeMicroseconds(1000);
  MOTOR2.writeMicroseconds(1000);
  MOTOR3.writeMicroseconds(1000);
  MOTOR4.writeMicroseconds(1000);

  while(micros() - loop_timer < 1000);
  loop_timer = micros();
}