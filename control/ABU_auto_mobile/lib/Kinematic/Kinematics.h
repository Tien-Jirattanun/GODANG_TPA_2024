#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "math.h"
class Kinematics {
private:
  float _MaxRADS = 12;
  float _Max_Vx = 0.76;
  float _Max_Vy = 0.76;
  float _Max_Wz = 2;

public:
  struct Robot {
    float wheel_diameter;
    float lx;
    float ly;
    float wheel_circumference;
    float angular_to_rpm;
  };

  Robot robot;

  struct RadPS {
    float radps_fl;
    float radps_fr;
    float radps_bl;
    float radps_br;
  };

  struct Velocity {
    float vx;
    float vy;
    float wz;
  };

  struct Position {
    double x;
    double y;
    double theta;  //in degree
  };

  Position current_position;

  Kinematics(float wheel_diameter, float lx, float ly);
  RadPS Inverse_Kinematics(float vx, float vy, float wz);
  Velocity Forward_Kinematics_Velocity(float radps_fl, float radps_fr, float radps_bl, float radps_br);
  Position Forward_Kinematics_Position(float radps_fl, float radps_fr, float radps_bl, float radps_br, Position current_position, float dt);
};

#endif