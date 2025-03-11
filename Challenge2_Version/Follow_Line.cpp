#include <Wire.h>
#include "Param.h"
#include <Zumo32U4.h>
#include "initialization.h"
#include "Follow_Line.h"

int integral = 0;

int LIM_SPEED = 2 * MIN_SPEED;

float Kp = 0.25;  // Proportional
float Ki = 0.0;   // Integral (adjust if necessary)
float Kd = 6.0;   // Derivative

int16_t lastError = 0;

void followLine(int16_t position) {
  int16_t error = position - 2000;
  integral += error;
  int16_t speedDifference = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
  lastError = error; 

  // Calculate motor speeds
  int16_t leftSpeed = (int16_t)LIM_SPEED + speedDifference;
  int16_t rightSpeed = (int16_t)LIM_SPEED - speedDifference;

  // Restrict speeds to avoid out-of-range values
  leftSpeed = constrain(leftSpeed, 0, (int16_t)LIM_SPEED);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)LIM_SPEED);

  // Apply speeds to the motors
  motors.setSpeeds(leftSpeed, rightSpeed);
}
