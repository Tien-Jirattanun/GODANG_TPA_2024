#ifndef BALL_GRIPPER_H
#define BALL_GRIPPER_H
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

class BallGripper {
public:
  BallGripper(int limit_switch, int INA, int INB, int stepPin, int dirPin, int servoPin);

  void motor(int pwm);
  void motor_stop();
  void limitswitch();
  void stepper_cw(); //up
  void stepper_ccw(); //down

  void setup();
  void preparing();
  void grab();
  void lift();
  void release();
  void shoot();

private:
  Servo s;
  int servoPin_;
  int INA_;
  int INB_;
  int stepPin_;
  int dirPin_;
  int limitSwitchPin_;

  static const int stepsPerRevolution = 3500;
  static const int speed_step_down_ = 500;

  int startSpeed = 1000;
  int endSpeed = 400;
  int accelerationSteps = 1000;

  AccelStepper stepper;
};

#endif