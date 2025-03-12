#include <Wire.h>
#include <Zumo32U4.h>

#define NUM_SENSORS 5

unsigned int lineSensorValues[NUM_SENSORS];

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;


void initSystem(){  
  lineSensors.initFiveSensors();
  
  Serial.begin(115200); 
  Serial1.begin(38400);

  uint16_t batteryLevel = readBatteryMillivolts();
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  
  calibrateSensors();
  
  Serial1.println("Initialization completed");
  motors.setSpeeds(100, 100);
}

void calibrateSensors() {
   // Wait 1 second and then calibrate the sensors while rotating
  delay(1000);
  for (uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-200 , 200 );
    } else {
      motors.setSpeeds(200 , -200 );
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void loop() {
  Serial1.print("Sensor 0 :");Serial1.println(lineSensorValues[0]);
  Serial1.print("Sensor 1 :");Serial1.println(lineSensorValues[1]);
  Serial1.print("Sensor 2 :");Serial1.println(lineSensorValues[2]);
  Serial1.print("Sensor 3 :");Serial1.println(lineSensorValues[3]);
  Serial1.print("Sensor 4 :");Serial1.println(lineSensorValues[4]);
  delay(500);
}
