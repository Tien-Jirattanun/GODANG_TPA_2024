#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(float wheel_diameter, float lx, float ly) {
  robot.wheel_diameter = wheel_diameter;
  robot.lx = lx;
  robot.ly = ly;
  robot.wheel_circumference = robot.wheel_diameter * M_PI;
  robot.angular_to_rpm = 60 / robot.wheel_circumference;
}

Kinematics::Velocity Kinematics::Forward_Kinematics_Velocity(float radps_fl, float radps_fr, float radps_bl, float radps_br) {
  Velocity basev;
  basev.vx = ((robot.wheel_diameter / 2.0) / 4.0) * (radps_fl + radps_fr + radps_bl + radps_br);
  basev.vy = ((robot.wheel_diameter / 2.0) / 4.0) * (-radps_fl + radps_fr + radps_bl - radps_br);
  basev.wz = ((robot.wheel_diameter / 2.0) / (4.0 * (robot.lx + robot.ly))) * (-radps_fl + radps_fr - radps_bl + radps_br);
  return basev;
}

Kinematics::RadPS Kinematics::Inverse_Kinematics(float vx, float vy, float wz) {
  vx = max(-_Max_Vx, min(vx, _Max_Vx));
  vy = max(-_Max_Vy, min(vy, _Max_Vy));
  wz = max(-_Max_Wz, min(wz, _Max_Wz));

  float rads_fl = (vx - vy - (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5);
  float rads_fr = (vx + vy + (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5);
  float rads_bl = (vx + vy - (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5);
  float rads_br = (vx - vy + (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5);

  float maxWheelRADS = max(abs(rads_fl), max(abs(rads_fr), max(abs(rads_bl), abs(rads_br))));

  float scaledVx, scaledVy, scaledWz;

  if (maxWheelRADS > _MaxRADS) {
    float scaleFactor = _MaxRADS / maxWheelRADS;
    scaledVx = vx * scaleFactor;
    scaledVy = vy * scaleFactor;
    scaledWz = wz * scaleFactor;
  } else {
    scaledVx = vx;
    scaledVy = vy;
    scaledWz = wz;
  }

  RadPS wheel_rads;
  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    wheel_rads.radps_fl = 0;
    wheel_rads.radps_fr = 0;
    wheel_rads.radps_bl = 0;
    wheel_rads.radps_br = 0;
  } else {
    wheel_rads.radps_fl = (scaledVx - scaledVy - (robot.lx + robot.ly) * scaledWz) / (robot.wheel_diameter * 0.5);
    wheel_rads.radps_fr = (scaledVx + scaledVy + (robot.lx + robot.ly) * scaledWz) / (robot.wheel_diameter * 0.5);
    wheel_rads.radps_bl = (scaledVx + scaledVy - (robot.lx + robot.ly) * scaledWz) / (robot.wheel_diameter * 0.5);
    wheel_rads.radps_br = (scaledVx - scaledVy + (robot.lx + robot.ly) * scaledWz) / (robot.wheel_diameter * 0.5);
  }
  return wheel_rads;
}

Kinematics::Position Kinematics::Forward_Kinematics_Position(float radps_fl, float radps_fr, float radps_bl, float radps_br, Position current_position, float dt) {
  Velocity basev = Forward_Kinematics_Velocity(radps_fl, radps_fr, radps_bl, radps_br);
  // float dt = 1 / 1000.0;
  current_position.x += basev.vx * dt;
  current_position.y += basev.vy * dt;
  current_position.theta += basev.wz * dt * (180.0 / M_PI);
  current_position.theta = fmod(current_position.theta, 360.0);

  if (current_position.theta < 0) {
    current_position.theta += 360.0;
  }

  return current_position;
}