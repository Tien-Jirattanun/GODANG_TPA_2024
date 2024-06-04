#include "ball_gripper.h"
#include <Arduino.h>

BallGripper::BallGripper(int limit_switch, int INA, int INB, int stepPin, int dirPin, int servoPin)
  : limitSwitchPin_(limit_switch),
    INA_(INA),
    INB_(INB),
    stepPin_(stepPin),
    dirPin_(dirPin),
    servoPin_(servoPin) {
}

void BallGripper::motor(int pwm) {
  if (pwm > 0) {
    analogWrite(INA_, 0);  // up
    analogWrite(INB_, pwm);
  } else {
    analogWrite(INA_, abs(pwm));
    analogWrite(INB_, 0);  // down
  }
}

void BallGripper::motor_stop() {
  analogWrite(INA_, 0);  // stop
  analogWrite(INB_, 0);
}

// limit debug
void BallGripper::limitswitch() {
  if (digitalRead(limitSwitchPin_) == 1) {
    Serial.println("1");
  } else {
    Serial.println("0");  // trig
  }
}

void BallGripper::stepper_cw() {
  digitalWrite(dirPin_, HIGH);

  for (int i = 0; i < stepsPerRevolution; i++) {
    int currentSpeed = startSpeed - ((startSpeed - endSpeed) * i / accelerationSteps);
    currentSpeed = max(currentSpeed, endSpeed);
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(currentSpeed);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(currentSpeed);
  }

  if (stepsPerRevolution > accelerationSteps) {
    for (int i = accelerationSteps; i < stepsPerRevolution; i++) {
      digitalWrite(stepPin_, HIGH);
      delayMicroseconds(endSpeed);
      digitalWrite(stepPin_, LOW);
      delayMicroseconds(endSpeed);
    }
  }
}

void BallGripper::stepper_ccw() {
  while (digitalRead(limitSwitchPin_) == 1) {
    digitalWrite(dirPin_, LOW);
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(speed_step_down_);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(speed_step_down_);
  }
}

// additional function for control
void BallGripper::preparing() {
  motor(-250);
  delay(900);
  motor_stop();
}

void BallGripper::grab() {  // grab the ball and keep high from the ground
  s.write(150);
  delay(500);
  motor(250);
  delay(1000);
  motor_stop();
  s.write(30);
  delay(500);
}

void BallGripper::shoot() {
  stepper_cw();
  delay(50);
  stepper_ccw();
}

void BallGripper::setup() {
  s.attach(servoPin_);
  pinMode(limitSwitchPin_, INPUT_PULLUP);
  pinMode(INA_, OUTPUT);
  pinMode(INB_, OUTPUT);
  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);

  stepper_ccw();

  s.write(150);
  delay(500);
  motor(250);
  delay(900);
  motor_stop();
  s.write(30);
  delay(500);
}