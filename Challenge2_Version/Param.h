#ifndef __PARAM__H__
  #define __PARAM__H__

const int MIN_SPEED = 60;
extern int LIM_SPEED; //Min speed = 41, Max speed = 400

const int THRESHOLD_HIGH = 800;
const int THRESHOLD_LOW = 200;

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 1;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

extern char buffer[10];

#endif
