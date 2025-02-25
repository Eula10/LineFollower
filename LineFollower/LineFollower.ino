#include <Wire.h>
#include <Zumo32U4.h>

#define maxSpeed 400
#define minSpeed 75 //41
#define NUM_SENSORS 5

#define THRESHOLD_HIGH 800  // High threshold
#define THRESHOLD_LOW 200    // Low threshold
  
#define RECUL_SPEED -150
#define TEMPSDERECUL 250

// These might need to be tuned for different motor types.
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200  // ms
#define TURN_DURATION     300  // ms

const int32_t turnSpeed = 150; // Vitesse de rotation
const int32_t ticksPerTurn = 2490; // Nombre de ticks pour 360°
const int32_t ticks45 = ticksPerTurn * 45 / 360;  // 45°
const int32_t ticks90 = ticksPerTurn * 90 / 360;  // 90°
const int32_t ticks120 = ticksPerTurn * 120 / 360; // 135°

int32_t targetTicks = ticks45;

// This is the motor speed limit
const uint16_t limSpeed = 2 * minSpeed;

bool useEmitters = true;
bool running = false;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

enum State {
    FOLLOW_LINE,
    BLACK_ZONE,
    WHITE_ZONE
};

enum WhiteState {
    initial,
    straight,
    turn_right,
    turn_left
};

WhiteState stateWhite = straight;
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

void turnAngleLeft(void)
{
    Serial1.print("ANGLE : ");Serial1.println(targetTicks);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    while (abs(encoders.getCountsRight()) < targetTicks)
    {
        Serial1.println(encoders.getCountsRight());
        motors.setSpeeds(-turnSpeed, turnSpeed); // Rotation sur place
    }

    motors.setSpeeds(0, 0); // Stop rotation

    if(targetTicks == ticks45)
    {
      targetTicks = ticks90;
    }
    else if(targetTicks == ticks90)
    {
      targetTicks = ticks120;
    }
    else if(targetTicks == ticks120)
    {
      targetTicks = ticks45;
    }
}

void turnAngleRight()
{
  
    Serial1.print("ANGLE : ");Serial1.println(targetTicks);
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    while (abs(encoders.getCountsRight()) < targetTicks)
    {
        //Serial1.println(encoders.getCountsRight());
        motors.setSpeeds(turnSpeed, -turnSpeed); // Rotation sur place
    }

    motors.setSpeeds(0, 0); // Stop rotation

    if(targetTicks == ticks45)
    {
      targetTicks = ticks90;
    }
    else if(targetTicks == ticks90)
    {
      targetTicks = ticks120;
    }
    else if(targetTicks == ticks120)
    {
      targetTicks = ticks45;
    }
}

void setup() {
  uint16_t batteryLevel = readBatteryMillivolts();
  
  Serial1.begin(38400);
  Serial.begin(9600);  
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
    //lineSensors.read(lineSensorValues);
    //Serial1.println(position);
   // Serial1.println();
  
    // Evaluate state change
      switch (currentState) {
          case FOLLOW_LINE:
              //Serial1.println("Follow line\n");
              followLine(position);
              if (allOnBlack()) {
                  Serial1.println("BLACK_ZONE\n");
                  currentState = BLACK_ZONE;
              }
              break;
  
          case BLACK_ZONE:
              motors.setSpeeds(limSpeed, limSpeed);  // Move forward in a straight line
              if (allOnWhite()) {
                  Serial1.println("White zone\n");
                  currentState = WHITE_ZONE;
              }
              break;
  
          case WHITE_ZONE:
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
  switch (stateWhite) {
    case initial:
      Serial1.println("init");
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      stateWhite = straight;
      break;
      
    case straight:
      if(lineSensorValues[2] > THRESHOLD_HIGH || lineSensorValues[3] > THRESHOLD_HIGH || lineSensorValues[4] > THRESHOLD_HIGH)
      {
        motors.setSpeeds(RECUL_SPEED, RECUL_SPEED);
        delay(TEMPSDERECUL);
        Serial1.println("turn_left");
        stateWhite = turn_left;
      }

      else if(lineSensorValues[0] > THRESHOLD_HIGH || lineSensorValues[1] > THRESHOLD_HIGH)
      {
        motors.setSpeeds(RECUL_SPEED, RECUL_SPEED);
        delay(TEMPSDERECUL);
        Serial1.println("turn_right");
        stateWhite = turn_right; 
      }
    break;

    case turn_right:
      turnAngleRight();
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      stateWhite = straight;
      Serial1.println("straight");
      break;

    case turn_left:
      turnAngleLeft();
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      stateWhite = straight;
      Serial1.println("straight");
      break;
}

}

void turn(){
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
    }
}
