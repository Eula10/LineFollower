#include <Wire.h>
#include "Param.h"
#include <Zumo32U4.h>
#include "initialization.h"
#include "Follow_Line.h"

int timer1s = 0;
volatile long long int timer500ms = 0;
int FM = 0;

bool Route = 0;

char char1;
char char2;
char char3;
char char4;

char char5;
char char6;
char char7;
char char8;

char CapCentralRead = 0;
char CapCentralRead2 = 0;
char CapCentralRead3 = 0;

int n = 0;

const int NUM_SENSORS = 5;
unsigned int lineSensorValues[NUM_SENSORS];

enum class State : uint8_t {
    FOLLOW_LINE,
    AVANCER_1s,
    DECODE,
    STRAIGHT,
    AVANCER_500ms,
    STOP
};

State currentState = State::FOLLOW_LINE;

void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();
    }
}

// Check if all sensors detect black (> 800)
bool AllOnBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] < THRESHOLD_HIGH) {
            return false;
        }
    }
    return true;
}

bool detectcode() {
  if(CapCentralRead == 0 &&  (lineSensorValues[2] > THRESHOLD_LOW))
  {
    CapCentralRead = 1;
    return true;
  }
  else if(CapCentralRead == 1 &&  (lineSensorValues[2] < THRESHOLD_HIGH))
  {
    CapCentralRead = 0;
    return false;
  }

  else
  {
    return false;
  }
  
}

bool detectcode2() {
  if(CapCentralRead2 == 0 &&  (lineSensorValues[1] > THRESHOLD_LOW))
  {
    CapCentralRead2 = 1;
    return true;
  }
  else if(CapCentralRead2 == 1 &&  (lineSensorValues[1] < THRESHOLD_HIGH))
  {
    CapCentralRead2 = 0;
    return false;
  }

  else
  {
    return false;
  }
  
}

bool detectcode3() {
  if(CapCentralRead3 == 0 &&  (lineSensorValues[3] > THRESHOLD_LOW))
  {
    CapCentralRead3 = 1;
    return true;
  }
  else if(CapCentralRead3 == 1 &&  (lineSensorValues[3] < THRESHOLD_HIGH))
  {
    CapCentralRead3 = 0;
    return false;
  }

  else
  {
    return false;
  }
  
}

void decoder() {
  if(detectcode() && !Route)
  {
    if(n<=7)
    {
      char1 = (char1 << 1) | (lineSensorValues[0] > THRESHOLD_LOW);
      char2 = (char2 << 1) | (lineSensorValues[4] > THRESHOLD_LOW);
      n++;
    }
    else if (n<16)
    {
      char3 = (char3 << 1) | (lineSensorValues[0] > THRESHOLD_LOW);
      char4 = (char4 << 1) | (lineSensorValues[4] > THRESHOLD_LOW);
      n++;
    }
  }

  else if(detectcode() && Route)
  {
    if(n<=7)
    {
      char5 = (char5 << 1) | (lineSensorValues[0] > THRESHOLD_LOW);
      char6 = (char6 << 1) | (lineSensorValues[4] > THRESHOLD_LOW);
      n++;
    }
    else if (n<16)
    {
      char7 = (char7 << 1) | (lineSensorValues[0] > THRESHOLD_LOW);
      char8 = (char8 << 1) | (lineSensorValues[4] > THRESHOLD_LOW);
      n++;
    }
  }
   
    return true;
}

// Check if all sensors detect white (< 200)
bool AllOnWhite() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] > THRESHOLD_LOW) {
            return false;
        }
    }
    return true;
}

void stop() {
  motors.setSpeeds(0, 0);
}

void moveForward() {
  motors.setSpeeds(120, 120);  // Avanza hacia adelante con velocidad 200
}

void setup() {
  initSystem();
}


void loop() {
    // Obtener posición de la línea
    int16_t position = lineSensors.readLine(lineSensorValues);
    
    switch (currentState) {
        case State::FOLLOW_LINE:
            followLine(position);
            if (AllOnWhite()) {
                Serial1.println("AVANCER_1s");
                moveForward();
                timer1s = millis();
                currentState = State::AVANCER_1s;
            }
            if (AllOnBlack()) {
           
              Serial1.println("AVANCER_500ms");
                currentState = State::AVANCER_500ms;
                moveForward();
                //delay(100);
                timer500ms = millis();
            }
            break;
    
        case State::AVANCER_1s: 
            if (!AllOnWhite()) {
              Serial1.println("DECODE");
                currentState = State::DECODE;
            } 
            else if ((millis() - timer1s) >= 500) {
              Serial1.println("STRAIGHT");
                currentState = State::STRAIGHT;
            } 
            break;

        case State::DECODE: 
            if(n<16)
            {
              decoder();
            }
            else
            {
              if(!Route){
                
                if(AllOnWhite())
                  {delay(250);
                    Serial1.print(char1);
                    Serial1.print(char2);
                    Serial1.print(char3);
                    Serial1.println(char4);
                    Serial1.println("FOLLOW_LINE");
                    currentState = State::FOLLOW_LINE;
                    n=0;
                    Route = 1;
                  }
                }
              else
              {
               if(AllOnWhite())
                  {delay(250);
                    Serial1.print(char5);
                    Serial1.print(char6);
                    Serial1.print(char7);
                    Serial1.println(char8);
                    Serial1.println("FOLLOW_LINE");
                    currentState = State::FOLLOW_LINE;
                    n=0;
                    Route = 1;
                  }
              }
            }
            break;

        case State::AVANCER_500ms: 
            if ((millis() - timer500ms) >= 500)  {
              Serial1.println("STOP");
                currentState = State::STOP;
            } 
            else if (!AllOnBlack()) {
              Serial1.println("FOLLOW_LINE");
              timer500ms = 0;
              currentState = State::FOLLOW_LINE;
            } 
            break;

        case State::STRAIGHT: 
            if (!AllOnWhite()) {
              Serial1.println("FOLLOW_LINE");
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
