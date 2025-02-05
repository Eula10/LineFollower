#include <Wire.h>
#include <Zumo32U4.h>

const int MAX_SPEED = 400;
const int MIN_SPEED = 41;
const int LIM_SPEED = 2 * MIN_SPEED;
const int NUM_SENSORS = 5;

const int THRESHOLD_HIGH = 800;
const int THRESHOLD_LOW = 200;

const int32_t ticksPerTurn = 2490; // Nombre de ticks pour 360°
const int angles[] = {45, 90, 120}; // Lista de ángulos
int angleIndex = 0;  // Índice para iterar entre los ángulos
int direction = 1;  // 1 para giro a la derecha, -1 para giro a la izquierda

bool useEmitters = true;
bool running = false;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

enum class State : uint8_t {
    FOLLOW_LINE,
    BLACK_ZONE,
    WHITE_ZONE
};

enum class WhiteState : uint8_t {
    TURN_RIGHT,
    TURN_LEFT
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

void followLine(int16_t position) {
  int16_t error = position - 2000;
  integral += error;
  int16_t speedDifference = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
  lastError = error; 

  // Calculate motor speeds
  int16_t leftSpeed = (int16_t)limSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)limSpeed - speedDifference;

  // Restrict speeds to avoid out-of-range values
  leftSpeed = constrain(leftSpeed, 0, (int16_t)limSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)limSpeed);

  // Apply speeds to the motors
  motors.setSpeeds(leftSpeed, rightSpeed);
}

// Check if all sensors detect black (> 800)
bool allOnBlack() {
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

void updateAngle() {
    angleIndex = (angleIndex + 1) % 3;  // Avanza cíclicamente entre 0, 1 y 2
}

void turnNextAngle(int direction) {
    int angle = angles[angleIndex] * direction;  // Aplica la dirección al ángulo
    turnAngle(angle);  // Llama a la función de giro
    updateAngle();  // Actualiza el índice para el próximo ángulo
}

void turnAngle(int angle){
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    int direction = (angle > 0) ? 1 : -1; // Determina el sentido del giro

    int angleTicks = ticksPerTurn * angle / 360;
    targetTicks = ticksPerTurn * angle / 360;
    while (abs(encoders.getCountsRight()) < angleTicks){
        motors.setSpeeds(direction * -turnSpeed, direction * turnSpeed); // Giro en el sentido indicado
    }
    
    motors.setSpeeds(0, 0); // Detener giro
}

void setup() {
  uint16_t batteryLevel = readBatteryMillivolts();
  
  Serial1.begin(38400);
  Serial.begin(9600);  
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  lineSensors.initFiveSensors();

  //calibrateSensors();

  // Initialize integral to 0
  integral = 0;
  lastSampleTime = 0;

  Serial1.write("START\n");
}

void loop() {
  void loop() {
    if (!running) {
        motors.setSpeeds(0, 0);
        return;
    }

    // Obtener posición de la línea
    int16_t position = lineSensors.readLine(lineSensorValues);

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
            if (lineSensorValues[2] > THRESHOLD_HIGH || lineSensorValues[3] > THRESHOLD_HIGH || lineSensorValues[4] > THRESHOLD_HIGH) {
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                Serial1.println("turn_left");
                stateWhite = WhiteState::TURN_LEFT;
            } 
            else if (lineSensorValues[0] > THRESHOLD_HIGH || lineSensorValues[1] > THRESHOLD_HIGH) {
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                Serial1.println("turn_right");
                stateWhite = WhiteState::TURN_RIGHT;
            }
            break;

        case WhiteState::TURN_LEFT:
        case WhiteState::TURN_RIGHT:
            int direction = (stateWhite == WhiteState::TURN_LEFT) ? -1 : 1;
            turnNextAngle(direction);  // Gira dependiendo de la dirección
            motors.setSpeeds(LIM_SPEED, LIM_SPEED);
            Serial1.println("straight");
            stateWhite = WhiteState::INITIAL;
            break;

        default:
            Serial1.println("Estado desconocido");
            break;
    }
}

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
    }
}
