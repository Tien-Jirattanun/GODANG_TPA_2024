#include <Arduino.h>

#include "ball_gripper.h"

#define limit_switch 04
#define INA 10
#define INB 11
#define stepPin 16
#define dirPin 17
#define servoPin 5

#define IR1 2
#define IR2 3
#define IR3 6

int lastState = -1;
int maniState = 0;

BallGripper bg(limit_switch, INA, INB, stepPin, dirPin, servoPin);

void setup()
{
}

void setup1()
{
  bg.setup();
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
}

void loop()
{
}

void loop1()
{
  if (maniState != lastState)
  {
    switch (maniState)
    {
      case 1:
        bg.preparing();
        break;
      case 2:
        bg.grab();
        break;
      case 3:
        bg.shoot();
        break;
      case 4:
        bg.stepper_cw();
        break;
      case 5:
        bg.stepper_ccw();
        break;
      default:
        break;
    }
    lastState = maniState;
  }
  else
  {
  }
}

/*

#include "ball_gripper.h"

#define limit_switch 04
#define INA 10
#define INB 11
#define stepPin 16
#define dirPin 17
#define servoPin 5

#define IR1 2
#define IR2 3
#define IR3 6

int lastState = -1;

BallGripper bg(limit_switch, INA, INB, stepPin, dirPin, servoPin);

void setup() {

  Serial.begin(115200);
  bg.setup();
  // pinMode(IR1, INPUT);
  // pinMode(IR2, INPUT);
  // pinMode(IR3, INPUT);
}

void loop() {
  if (Serial.available()) {
    String state = Serial.readStringUntil('\n');

    int intState = state.toInt();

    if (intState != lastState) {
      switch (intState) {
        case 1:
          bg.preparing();

          break;
        case 2:
          bg.grab();

          break;
        case 3:
          bg.shoot();

          break;
        case 4:
          bg.stepper_cw();

          break;
        case 5:
          bg.stepper_ccw();

          break;
        default:
          //function

          break;
      }
      lastState = intState;
    } else {
    }
  }

  //Send IR State
  Serial.print(digitalRead(IR1));
  Serial.print(",");
  Serial.print(digitalRead(IR2));
  Serial.print(",");
  Serial.println(digitalRead(IR3));
}


*/