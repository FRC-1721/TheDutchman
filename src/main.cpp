#include <Arduino.h>
#include <Servo.h>

const int potPin = 4;
const int switchPin = 5;
const int PWMPin = 16;

// PWM Calibration for Kraken X60
const int REV_MAX = 1000; // 1.0 ms
const int NEUTRAL = 1500; // 1.5 ms
const int FORWARD_MAX = 2000; // 2.0 ms

// DEADBAND range
const int DEADBAND = 100;

// RAMP
const float RAMP_RATE = 2.5;

Servo KrakenServo;
float currentPWM = NEUTRAL;

void setup() {
  Serial.begin(115200);
  // Wait for a serial to initialize
  while (!Serial && millis() < 2000);
  
  pinMode(switchPin, INPUT_PULLDOWN);

  KrakenServo.attach(PWMPin, 1000, 2000);
  KrakenServo.writeMicroseconds(NEUTRAL);
}

void loop() {
  // put your main code here, to run repeatedly:
  int potValue = analogRead(potPin);
  int switchState = digitalRead(switchPin);
  
  // Calculate target
  int targetPWM = map(potValue, 0, 4095, REV_MAX, FORWARD_MAX);

  // Apply Deadband
  if (abs(targetPWM - NEUTRAL) < DEADBAND) targetPWM = NEUTRAL;

  // Limit Switch
  if (switchState == LOW) targetPWM = NEUTRAL;
  
  // Ramping
  if (currentPWM < targetPWM) {
    currentPWM += RAMP_RATE;
    if (currentPWM > targetPWM) currentPWM = targetPWM;
  } else if (currentPWM > targetPWM) {
    currentPWM -= RAMP_RATE;
    if (currentPWM < targetPWM) currentPWM = targetPWM;
  }

  KrakenServo.writeMicroseconds((int)currentPWM);

  // Debugging
  static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) { // Print every 100ms
        Serial.print("Target: "); Serial.print(targetPWM);
        Serial.print("us | Current: "); Serial.print((int)currentPWM);
        Serial.println(currentPWM == NEUTRAL ? " [IDLE]" : "");
        lastPrint = millis();
    }

  delay(20);
}