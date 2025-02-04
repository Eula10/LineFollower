#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4Motors motors;

uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

char report[80];

void setup() {
  Serial1.begin(9600);
  turnSensorSetup();
  delay(500);
  turnSensorReset();
  turnSensorUpdate();
  if(turnAngle < turnAngle45 ){
    motors.setSpeeds(80, -80);
  }
  motors.setSpeeds(0, 0);
}

void loop() {
  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;

  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    if (errorLeft)
    {
      // An error occurred on the left encoder channel.
      // Show it on the display for the next 10 iterations and
      // also beep.
      displayErrorLeftCountdown = 10;
    }

    if (errorRight)
    {
      // An error occurred on the right encoder channel.
      // Show it on the display for the next 10 iterations and
      // also beep.
      displayErrorRightCountdown = 10;
    }

    if (displayErrorLeftCountdown)
    {
      // Show an exclamation point on the first line to
      // indicate an error from the left encoder.
      displayErrorLeftCountdown--;
    }

    if (displayErrorRightCountdown)
    {
      // Show an exclamation point on the second line to
      // indicate an error from the left encoder.
      displayErrorRightCountdown--;
    }

    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %1d %1d"),
        countsLeft, countsRight, errorLeft, errorRight);
    Serial.println(report);
  }

}
