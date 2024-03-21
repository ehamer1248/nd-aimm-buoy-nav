#include <Arduino.h>

// Motor Direction pins
#define DIR1_PIN 12
#define DIR2_PIN 13

// Motor PWM pins
#define PWM1_PIN 10
#define PWM2_PIN 11

void setup() {
  // Initialize the direction pins as outputs
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  // Initialize the PWM pins as outputs
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  // Ensure motor is stopped initially
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    int commaIndex1 = command.indexOf(',');
    int commaIndex2 = command.indexOf(',', commaIndex1 + 1);

    if (commaIndex1 != -1 && commaIndex2 != -1) {
      String directionStr = command.substring(0, commaIndex1);
      String pwm1Str = command.substring(commaIndex1 + 1, commaIndex2);
      String pwm2Str = command.substring(commaIndex2 + 1);

      int pwm1Speed = pwm1Str.toInt();
      int pwm2Speed = pwm2Str.toInt();

      if (directionStr == "forward") {
        digitalWrite(DIR1_PIN, HIGH);
        digitalWrite(DIR2_PIN, HIGH);
        analogWrite(PWM1_PIN, pwm1Speed);
        analogWrite(PWM2_PIN, pwm2Speed);
      } else if (directionStr == "left") {
        digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, HIGH);
        analogWrite(PWM1_PIN, pwm1Speed);
        analogWrite(PWM2_PIN, pwm2Speed);
      } else if (directionStr == "right") {
        digitalWrite(DIR1_PIN, HIGH);
        digitalWrite(DIR2_PIN, LOW);
        analogWrite(PWM1_PIN, pwm1Speed);
        analogWrite(PWM2_PIN, pwm2Speed);
      } else if (directionStr == "brake") {
        // Reverse the motors for 3 seconds
        digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, LOW);
        analogWrite(PWM1_PIN, pwm1Speed);
        analogWrite(PWM2_PIN, pwm2Speed);
        delay(3000);
        digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, LOW);
        analogWrite(PWM1_PIN, 0);
        analogWrite(PWM2_PIN, 0);
        
        // Stop the motors
        analogWrite(PWM1_PIN, 0);
        analogWrite(PWM2_PIN, 0);
      } else if (directionStr == "stop") {
        digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, LOW);
        analogWrite(PWM1_PIN, 0);
        analogWrite(PWM2_PIN, 0);
      }
    }
  }
}