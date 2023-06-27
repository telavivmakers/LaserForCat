#include <Arduino.h>
#include <Servo.h>

Servo yawServo;
Servo pitchServo;
int start_h_angle_pin = A0;
int range_h_angle_pin = A1;
int start_w_angle_pin = A2;
int range_w_angle_pin = A3;
int laserPin = 4;

int modePin = 5;
int mode;

int h_angle;
int w_angle;
int start_h_angle;
int start_w_angle;
int range_h_angle;
int range_w_angle;

int h_shift;
int w_shift;

int state = 0;
int shift = 1;
int speed_factor = 1;

void setup() {
  pinMode(start_h_angle_pin, INPUT);
  pinMode(range_h_angle_pin, INPUT);
  pinMode(start_w_angle_pin, INPUT);
  pinMode(range_w_angle_pin, INPUT);
  pinMode(laserPin, OUTPUT);
  yawServo.attach(9);
  pitchServo.attach(10);

  mode = digitalRead(modePin);

  h_angle = start_h_angle;
  w_angle = start_w_angle;
  
  Serial.begin(9600);
}

void loop() {
  mode = digitalRead(modePin);

  start_h_angle = map(analogRead(start_h_angle_pin), 0, 1023, 0, 180);
  delay(2);
  range_h_angle = map(analogRead(range_h_angle_pin), 0, 1023, 0, 180 - start_h_angle);
  delay(2);
  start_w_angle = map(analogRead(start_w_angle_pin), 0, 1023, 0, 180);
  delay(2);
  range_w_angle = map(analogRead(range_w_angle_pin), 0, 1023, 0, 180 - start_w_angle);
  delay(2);

  digitalWrite(laserPin, HIGH);

  if (mode == 0) {
    switch (state) {
      case 0:
        if (h_angle < (start_h_angle + range_h_angle)) {
          h_angle += shift;
        }
        else {
          state = 1;
        }
        break;
      case 1:
        if (w_angle < (start_w_angle + range_w_angle)) {
          w_angle += shift;
        }
        else {
          state = 2;
        }
        break;
      case 2:
        if (h_angle > start_h_angle) {
          h_angle -= shift;
        }
        else {
          state = 3;
        }
        break;
      case 3:
        if (w_angle > start_w_angle) {
          w_angle -= shift;
        }
        else {
          state = 0;
        }
        break;
    }

    h_shift = 0;
    w_shift = 0;
  }
  else {
    h_shift = random(-1, 1);
    w_shift = random(-1, 1);
    
    h_angle += h_shift;
    w_angle += w_shift;

    if ((h_angle > (start_h_angle + range_h_angle)) || (h_angle < start_h_angle)) {
      h_angle = start_h_angle + int(range_h_angle / 2);
    }
    
    if ((w_angle > (start_w_angle + range_w_angle)) || (w_angle < start_w_angle)) {
      w_angle = start_w_angle + int(range_w_angle / 2);
    }
  }

  pitchServo.write(h_angle);
  yawServo.write(w_angle);

  Serial.print("start height: ");
  Serial.print(start_h_angle);
  Serial.print(" (range: ");
  Serial.print(range_h_angle);
  Serial.print(")");
  
  Serial.print("; start width: ");
  Serial.print(start_w_angle);
  Serial.print(" (range: ");
  Serial.print(range_w_angle);
  Serial.print(")");
  
  Serial.print("; shift: ");
  Serial.print(h_shift);
  Serial.print(", ");
  Serial.println(w_shift);

  delay(10 * speed_factor);
}