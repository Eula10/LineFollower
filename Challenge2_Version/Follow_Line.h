#ifndef __FOLLOW__LINE__H__
  #define __FOLLOW__LINE__H__

  extern int integral;
  void followLine(int16_t position);

    // Define PID constants for easy modification
  extern float Kp;  // Proportional
  extern float Ki;   // Integral (adjust if necessary)
  extern float Kd;   // Derivative

  extern int16_t lastError; // Last error for derivative calculation

#endif
