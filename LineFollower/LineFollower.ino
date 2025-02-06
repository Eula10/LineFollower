#include <Wire.h>
#include <Zumo32U4.h>

const int MIN_SPEED = 41;
int LIM_SPEED = 2 * MIN_SPEED; //Min speed = 41, Max speed = 400
const int NUM_SENSORS = 5;
const int REVERSE_DURATION = 500;

const int THRESHOLD_HIGH = 800;
const int THRESHOLD_LOW = 200;

const int32_t ticksPerTurn = 2490; // Nombre de ticks pour 360°
const int angles[] = {45, 90, 120}; // Lista de ángulos
int angleIndex = 0;  // Índice para iterar entre los ángulos
int commandAngle = 4;

int direction = 1;  // 1 para giro a la derecha, -1 para giro a la izquierda

bool useEmitters = true;
bool running = false;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;

enum class State : uint8_t {
    FOLLOW_LINE,
    BLACK_ZONE,
    WHITE_ZONE,
    TURN_RIGHT,
    TURN_LEFT,
    COMBAT_MODE
};

volatile State currentState = State::FOLLOW_LINE;

// Define PID constants for easy modification
float Kp = 0.25;  // Proportional
float Ki = 0.0;   // Integral (adjust if necessary)
float Kd = 6.0;   // Derivative

int16_t integral = 0;  // Accumulator for the integral term
int16_t lastError = 0; // Last error for derivative calculation

char buffer[10];

static uint16_t lastSampleTime = 0;

unsigned int lineSensorValues[NUM_SENSORS];

//Combat Mode
const uint8_t sensorThreshold = 1;
const uint16_t turnSpeedMax = 150;
const uint16_t turnSpeedMin = 100;
const uint16_t deceleration = 10;
const uint16_t acceleration = 10;

#define LEFT 0
#define RIGHT 1

bool senseDir = RIGHT;
uint16_t turnSpeed = turnSpeedMax;



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

// Check if all sensors detect black (> 800)
bool allOnBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] < THRESHOLD_HIGH) return false;
    }
    return true;
}

// Check if all sensors detect white (< 200)
bool allOnWhite() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] > THRESHOLD_LOW) return false;
    }
    return true;
}

bool detectEdge() {
  for (int i = 0; i < NUM_SENSORS; i++) {
      if (lineSensorValues[i] > THRESHOLD_HIGH) return true;  // Si detectamos un borde, salimos del bucle
  }
  return false;
}

void updateAngle() {
    if (commandAngle >= 1 && commandAngle <= 3) {
        angleIndex = commandAngle - 1;  // Ajusta el índice para 0, 1, 2
    } else if (commandAngle == 4) {
        // Reinicia el ciclo de ángulos (itera cíclicamente entre 45, 90, 120)
        angleIndex = (angleIndex + 1) % 3;
    }
    Serial1.print("Angle Index = ");
    Serial1.println(angleIndex);
}

void stop() {
  motors.setSpeeds(0, 0);
}

void moveForward() {
  motors.setSpeeds(200, 200);  // Avanza hacia adelante con velocidad 200
}

void turnNextAngle(int direction) {
    int angle = angles[angleIndex] * direction;  // Aplica la dirección al ángulo
    turnAngle(angle, direction);  // Pasa la dirección a turnAngle
    updateAngle();  // Actualiza el índice para el próximo ángulo
}

void turnAngle(int angle, int direction){
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    int angleTicks = ticksPerTurn * abs(angle) / 360;  // Usa el valor absoluto de angle
    
    // Asegura que el giro se haga en la dirección correcta
    while (abs(encoders.getCountsRight()) < angleTicks){
        motors.setSpeeds(direction * -LIM_SPEED, direction * LIM_SPEED); // Dirección controlada por el parámetro 'direction'
    }

    motors.setSpeeds(0, 0); // Detener motores después del giro
}  

void setup() {
  uint16_t batteryLevel = readBatteryMillivolts();
  
  Serial1.begin(38400);
  Serial.begin(9600);  
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  calibrateSensors();

  // Initialize integral to 0
  integral = 0;
  lastSampleTime = 0;

  Serial1.write("START\n");
}

void loop() {
    if (!running) {
        motors.setSpeeds(0, 0);
        return;
    }
    
    // Obtener posición de la línea
    int16_t position = lineSensors.readLine(lineSensorValues);

    Serial1.print("currentState = ");
    Serial1.println(static_cast<uint8_t>(currentState));
    if (currentState == State::COMBAT_MODE) {
    Serial1.println("Entrando a COMBAT_MODE antes del switch");
}

    switch (currentState) {
        case State::FOLLOW_LINE:
            followLine(position);
            if (allOnBlack()) {
                Serial1.println("BLACK_ZONE");
                currentState = State::BLACK_ZONE;
            }
            break;
    
        case State::BLACK_ZONE:
            motors.setSpeeds(LIM_SPEED, LIM_SPEED);  // Avanzar en línea recta
            if (allOnWhite()) {
                Serial1.println("WHITE_ZONE");
                currentState = State::WHITE_ZONE;
            }
            break;
    
        case State::WHITE_ZONE:
            // Comprobar si es necesario girar a la izquierda o derecha
            if (lineSensorValues[2] > THRESHOLD_HIGH || lineSensorValues[3] > THRESHOLD_HIGH || lineSensorValues[4] > THRESHOLD_HIGH) {
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                delay(REVERSE_DURATION);
                Serial1.println("turn_left");
                currentState = State::TURN_LEFT;  // Cambiar a giro a la izquierda
            } 
            else if (lineSensorValues[0] > THRESHOLD_HIGH || lineSensorValues[1] > THRESHOLD_HIGH) {
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                delay(REVERSE_DURATION);
                Serial1.println("turn_right");
                currentState = State::TURN_RIGHT;  // Cambiar a giro a la derecha
            }
            break;
    
        case State::TURN_LEFT:
        case State::TURN_RIGHT:
            int direction = (currentState == State::TURN_LEFT) ? 1 : -1;
            turnNextAngle(direction);  // Gira dependiendo de la dirección
            motors.setSpeeds(LIM_SPEED, LIM_SPEED);
            Serial1.println("Moving Forward");
            currentState = State::WHITE_ZONE;  // Resetea el estado después de girar
            break;

        case State::COMBAT_MODE:
          Serial1.println("Combat Mode On");
          Serial1.print("Edge = ");
//            if (detectEdge()) {
//                 motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
//                delay(REVERSE_DURATION);
//                turnAngle(180,1);
//                motors.setSpeeds(LIM_SPEED, LIM_SPEED);
//                delay(REVERSE_DURATION);                
//            }
//            proxSensors.read();
//            uint8_t leftValue = proxSensors.countsFrontWithLeftLeds();
//            Serial1.print("leftValue = ");
//            Serial1.println(leftValue);
//            uint8_t rightValue = proxSensors.countsFrontWithRightLeds();
//            Serial1.print("rightValue = ");
//            Serial1.println(rightValue);
//
//            // Determine if an object is visible or not.
//            bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;
//            Serial1.print("objectSeen = ");
//            Serial1.println(objectSeen);
//
//              if (objectSeen) {
//                  // Adjust turn speed based on whether an object is detected
//                  turnSpeed = (leftValue < rightValue) ? turnSpeedMax - deceleration : (leftValue > rightValue) ? turnSpeedMax - deceleration : turnSpeedMax;
//                  
//                  if (leftValue < rightValue) {
//                    motors.setSpeeds(turnSpeed, -turnSpeed);  // Turn right
//                    senseDir = RIGHT;
//                  } 
//                  else if (leftValue > rightValue) {
//                    motors.setSpeeds(-turnSpeed, turnSpeed);  // Turn left
//                    senseDir = LEFT;
//                  } 
//                  else {
//                    // Both sensors detect equally, move forward
//                    moveForward();
//                  }
//                }
//                else {
//                  // No object detected, continue turning in the last sensed direction
//                  if (senseDir == RIGHT) {
//                    motors.setSpeeds(turnSpeed, -turnSpeed);  // Turn right
//                  } 
//                  else {
//                    motors.setSpeeds(-turnSpeed, turnSpeed);  // Turn left
//                  }
//                }
            break;
    
        default:
            Serial1.println("indefinido");
            break;
    }
}

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
        if (receivedChar == 'B') LIM_SPEED = 400;
        if (receivedChar == 'C') currentState = State::COMBAT_MODE;
        Serial1.print("Recibido: ");
        Serial1.println(receivedChar);
        Serial1.print("Nuevo estado: ");
        Serial1.println(static_cast<uint8_t>(currentState));
        if (receivedChar == 'D') currentState = State::WHITE_ZONE;
        if (receivedChar == 'N') LIM_SPEED = 2 * MIN_SPEED;
        if (receivedChar == '1') commandAngle = 1;
        if (receivedChar == '2') commandAngle = 2;
        if (receivedChar == '3') commandAngle = 3;
        if (receivedChar == '4') commandAngle = 4;
    }
}
