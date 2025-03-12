#include <Zumo32U4.h>

#ifndef __INITIALIZATION__H__
  #define __INITIALIZATION__H__

  extern Zumo32U4LineSensors lineSensors;
  extern Zumo32U4Motors motors;
  void initSystem();
  void calibrateSensors();


#endif
