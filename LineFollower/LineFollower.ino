#include <Wire.h>
#include <Zumo32U4.h>

#define maxSpeed 400
#define minSpeed 41
#define NUM_SENSORS 5

#define THRESHOLD_HIGH 800  // High threshold
#define THRESHOLD_LOW 200    // Low threshold

// This is the motor speed limit
const uint16_t limSpeed = 2 * minSpeed;

bool useEmitters = true;
bool running = false;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;

enum State {
    FOLLOW_LINE,
    BLACK_ZONE,
    WHITE_ZONE
};

State currentState = FOLLOW_LINE;

// Define PID constants for easy modification
float Kp = 0.25;  // Proportional
float Ki = 0.0;   // Integral (adjust if necessary)
float Kd = 6.0;   // Derivative

int16_t integral = 0;  // Accumulator for the integral term
int16_t lastError = 0; // Last error for derivative calculation

char buffer[10];

static uint16_t lastSampleTime = 0;

unsigned int lineSensorValues[NUM_SENSORS];



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

void setup() {
  uint16_t batteryLevel = readBatteryMillivolts();
  
  Serial1.begin(9600);
  Serial.begin(9600);
  sprintf(buffer, "%u", batteryLevel);  
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  lineSensors.initFiveSensors();

  // Wait until button C is pressed before starting
  //buttonC.waitForButton();

  calibrateSensors();

  // Initialize integral to 0
  integral = 0;
  lastSampleTime = 0;

  Serial1.write("START\n");
}

void loop() {
  if (running) {
    // Get the line position
    int16_t position = lineSensors.readLine(lineSensorValues);
    Serial1.println(position);
    Serial1.println();
  
    // Evaluate state change
      switch (currentState) {
          case FOLLOW_LINE:
              Serial1.println("Follow line\n");
              followLine(position);
              if (allOnBlack()) {
                  currentState = BLACK_ZONE;
              }
              break;
  
          case BLACK_ZONE:
              Serial1.println("Black zone\n");
              motors.setSpeeds(limSpeed, limSpeed);  // Move forward in a straight line
              if (allOnWhite()) {
                  currentState = WHITE_ZONE;
              }
              break;
  
          case WHITE_ZONE:
              Serial1.println("White zone\n");
              stayInWhiteZone();
              break;
      }
  //    delay(500);
  } else {
        motors.setSpeeds(0, 0);  
    }
}

void followLine(int16_t position) {
  // Calculate the error (deviation from the center)
  int16_t error = position - 2000;

  // Calculate the integral term (adjust Ki before using it)
  integral += error;

  // Calculate the PID correction
  int16_t speedDifference = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));

  lastError = error; // Save the current error for the next derivative calculation

  // Calculate motor speeds
  int16_t leftSpeed = (int16_t)limSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)limSpeed - speedDifference;

  // Restrict speeds to avoid out-of-range values
  leftSpeed = constrain(leftSpeed, 0, (int16_t)limSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)limSpeed);

  // Apply speeds to the motors
  motors.setSpeeds(leftSpeed, rightSpeed);
}

// Check if all sensors detect black (> 1000)
bool allOnBlack() {
//  char buffer[80];
//  sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
//    lineSensorValues[0],
//    lineSensorValues[1],
//    lineSensorValues[2],
//    lineSensorValues[3],
//    lineSensorValues[4],
//    useEmitters ? 'E' : 'e'
//  );
//  Serial1.print(buffer);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] < THRESHOLD_HIGH) {
            return false;
        }
    }
    return true;
}

// Check if all sensors detect white (< 200)
bool allOnWhite() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] > THRESHOLD_LOW) {
            return false;
        }
    }
    return true;
}

void stayInWhiteZone() {
    if (allOnBlack()) {
      turn();
    }
    else {
        // Evaluate if the line is in the center, right, or left
        if (lineSensorValues[0] > THRESHOLD_HIGH) {
            turn();
        }
        else if (lineSensorValues[4] > THRESHOLD_HIGH) {
            turn();
        }
        else {
            motors.setSpeeds(limSpeed, limSpeed);
        }
    }
}

void turn(){
  }

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
    }
}
