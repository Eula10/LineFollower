#include <Wire.h>
#include <Param.h>
#include <Zumo32U4.h>
#include <initialization.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;

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
    if (commandAngle >= 1 && commandAngle <= 3) {
        angleIndex = commandAngle - 1;  // Ajusta el índice para 0, 1, 2
    } else if (commandAngle == 4) {
        // Reinicia el ciclo de ángulos (itera cíclicamente entre 45, 90, 120)
        angleIndex = (angleIndex + 1) % 3;
    }
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

void stop() {
  motors.setSpeeds(0, 0);
}

void moveForward() {
  motors.setSpeeds(200, 200);  // Avanza hacia adelante con velocidad 200
}


void setup() {
  initSystem();

}

int timer1s = 0;
int timer500ms = 0;

void loop() {
    if (running) {
       // motors.setSpeeds(0, 0);
        //return;
    }
    
    // Obtener posición de la línea
    int16_t position = lineSensors.readLine(lineSensorValues);
    
    switch (currentState) {
        case State::FOLLOW_LINE:
            followLine(position);
            if (allOnWhite()) {
                moveForward();
                timer1s = millis();
                currentState = State::AVANCER_1s;
            }
            if (allOnBlack()) {
                currentState = State::AVANCER_500ms;
                timer500ms = millis();
            }
            break;
    
        case State::AVANCER_1s: 
            if (!AllOnWhite()) {
                currentState = State::DECODE;
            } 
            else if ( ((timer1s - (millis())) >= 1000 ) ) {
                currentState = State::STRAIGHT;
            } 
            break;

        case State::DECODE: 
            if (FM>=16 && AllOnWhite()) {
                currentState = State::FOLLOW_LINE;
            }
            break;

        case State::AVANCER_500ms: 
            if ( ((timer500ms - (millis())) >= 500 ) ) {
                currentState = State::STOP;
            } 
            else if (!AllOnBlack())) {
                currentState = State::FOLLOW_LINE;
            } 
            break;

        case State::STRAIGHT: 
            if (!AllOnWhite()) {
                currentState = State::FOLLOW_LINE;
            }
            break;

         case State::STOP: 
            stop();
            break;
            
       default:
            //Serial1.println("White State");
            break;
    }
}


void serialEvent1() {                                             //Data in Serial1
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();
    }
}
