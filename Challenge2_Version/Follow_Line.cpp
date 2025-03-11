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
