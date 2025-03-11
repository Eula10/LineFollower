#include <Wire.h>
#include <Param.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
Zumo32U4ProximitySensors proxSensors

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
  
  proxSensors.initFrontSensor();
  uint16_t batteryLevel = readBatteryMillivolts();
  
  Serial1.begin(38400);
  Serial.begin(9600);  
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  lineSensors.initFiveSensors();

  calibrateSensors();

  // Initialize integral to 0
  integral = 0;
  lastSampleTime = 0;

  Serial1.write("START\n");
}

void loop() {
    if (running) {
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
              startTime = millis();
              endTime = millis();
              Serial1.print("StartTime = ");
              Serial1.println(startTime/1000);
              encoders.getCountsAndResetLeft();
              encoders.getCountsAndResetRight();
              motors.setSpeeds(LIM_SPEED, LIM_SPEED);  // Avanzar en línea recta
              buzzer.play("L16 cdegreg4");
              delay(1000);
              currentState = State::MEASUREMENT;
            break;

        case State::MEASUREMENT:
            if (lineSensorValues[2] > THRESHOLD_HIGH || lineSensorValues[3] > THRESHOLD_HIGH || lineSensorValues[4] > THRESHOLD_HIGH)
            {
              distance = encoders.getCountsRight();
              Serial1.print("Distance = ");
              Distance = (distance * 4 * 3.141592)/ 909;
              Serial1.print(Distance);
              Serial1.println(" cm");
              currentState = State::WHITE_ZONE;
              buzzer.play(">g32>>c32");
            }
            else if (lineSensorValues[0] > THRESHOLD_HIGH || lineSensorValues[1] > THRESHOLD_HIGH)
            {
              distance = encoders.getCountsRight();
              Serial1.print("Distance = ");
              Distance = (distance * 4 * 3.141592)/ 909;
              Serial1.print(Distance);
              Serial1.println(" cm");
              currentState = State::WHITE_ZONE;
              buzzer.play(">g32>>c32");
            }
        
            break;
    
        case State::WHITE_ZONE: 
            Serial1.print("In white Zone");           
            // Comprobar si es necesario girar a la izquierda o derecha
            if (lineSensorValues[2] > THRESHOLD_HIGH || lineSensorValues[3] > THRESHOLD_HIGH || lineSensorValues[4] > THRESHOLD_HIGH) {
                Serial1.print("Try to turn Left");
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                delay(REVERSE_DURATION);
                Serial1.print("currentState como número: ");
                Serial1.println(static_cast<int>(currentState));
                currentState = State::TURN_LEFT;  // Cambiar a giro a la izquierda
                Serial1.print("currentState como número: ");
                Serial1.println(static_cast<int>(currentState));
            } 
            else if (lineSensorValues[0] > THRESHOLD_HIGH || lineSensorValues[1] > THRESHOLD_HIGH) {
                Serial1.print("Try to turn Rigth");
                motors.setSpeeds(-LIM_SPEED, -LIM_SPEED);
                delay(REVERSE_DURATION);
                Serial1.print("currentState como número: ");
                Serial1.println(static_cast<int>(currentState));
                currentState = State::TURN_RIGHT;  // Cambiar a giro a la derecha
                Serial1.print("currentState como número: ");
                Serial1.println(static_cast<int>(currentState));
            }

            // Read the front proximity sensors
//            proxSensors.read();
//            Serial1.print("Reading Sensor");
//            uint8_t leftValue = 0;//proxSensors.countsFrontWithLeftLeds();
//            uint8_t rightValue = 0;//proxSensors.countsFrontWithRightLeds();
////            Serial1.print("end reading sensor");
//
//            // Determine if an object is visible or not.
//            bool objectSeen = leftValue >= sensorThreshold || rightValue >= sensorThreshold;
          
//            if (objectSeen) {
//              Serial1.print("Object detected");
//              // Adjust turn speed based on whether an object is detected
//              turnSpeed = (leftValue < rightValue) ? turnSpeedMax - deceleration : (leftValue > rightValue) ? turnSpeedMax - deceleration : turnSpeedMax;
//              
//              if (leftValue < rightValue) {
//                motors.setSpeeds(turnSpeed, -turnSpeed);  // Turn right
//                senseDir = RIGHT;
//              } 
//              else if (leftValue > rightValue) {
//                motors.setSpeeds(-turnSpeed, turnSpeed);  // Turn left
//                senseDir = LEFT;
//              } 
//              else {
//                // Both sensors detect equally, move forward
//                moveForward();
//              }
//            }
//            else {
//              // No object detected, continue turning in the last sensed direction
//              if (senseDir == RIGHT) {
//                motors.setSpeeds(turnSpeed, -turnSpeed);  // Turn right
//              } 
//              else {
//                motors.setSpeeds(-turnSpeed, turnSpeed);  // Turn left
//              }
//            }


            
            break;



        case State::TURN_LEFT:
        case State::TURN_RIGHT:
            Serial1.print("currentState como número: ");
            Serial1.println(static_cast<int>(currentState));
            int direction = (currentState == State::TURN_LEFT) ? 1 : -1;
            turnNextAngle(direction);  // Gira dependiendo de la dirección
            motors.setSpeeds(LIM_SPEED, LIM_SPEED);
            currentState = State::WHITE_ZONE;  // Resetea el estado después de girar
            Serial1.print("End of the turn");
            break;
    
        default:
            //Serial1.println("White State");
            break;
    }
    
    if (endTime - startTime >= 150000) {  // 3 minutes in milliseconds
      buzzer.play("L16 c e g c5");
      Serial1.print("Time = ");
      Serial1.print((endTime - startTime)/1000);
      Serial1.println(" s");
      motors.setSpeeds(0, 0);  // Stop the robot
      Serial1.println("Stopping the robot.");
      currentState = State::STOP;
    }else{
      endTime = millis();
      if ((endTime - startTime) % 10000 == 0) {
          Serial1.print("Time = ");
          Serial1.print((endTime - startTime)/ 1000);
          Serial1.println(" s");
      }
    }
}

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
        if (receivedChar == 'B') LIM_SPEED = 300;
        if (receivedChar == 'N') LIM_SPEED = 2 * MIN_SPEED;
        if (receivedChar == '1') commandAngle = 1;
        if (receivedChar == '2') commandAngle = 2;
        if (receivedChar == '3') commandAngle = 3;
        if (receivedChar == '4') commandAngle = 4;
    }
}
