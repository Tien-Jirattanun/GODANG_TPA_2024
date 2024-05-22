#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "pio_encoder.h"
// #include "LowPass.h"

class Motor {
public:
  Motor(int encoderPin, int INA_PIN, int INB_PIN, double kp, double ki, double kd);  // Constructor
  void setSpeed(int speed);                                                          // Set the speed of the motor
  void stop();                                                                       // Stop the motor
  float computeRADS(float setPoint_RADS, float deltaTime);                           // Compute the PID and update speed
  void FindSpeedFromPWM(float pwm, float deltaTime);

  PioEncoder encoder;

private:
  // LowPass<2> lp;
  float _vFilt;
  float _v, _u, _ffw;
  int _INA_PIN;
  int _INB_PIN;
  float _kp, _ki, _kd;
  float _vPrev;
  float _error, _lastError, _integral;
  int _prevCount = 0;
};

#endif