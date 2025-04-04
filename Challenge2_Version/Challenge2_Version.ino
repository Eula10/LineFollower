#include <Wire.h>
#include "Param.h"
#include <Zumo32U4.h>
#include "initialization.h"
#include "Follow_Line.h"

#define vitesse_normale 100
#define diff_vitesse    20

volatile long long int timer1s = 0;
volatile long long int timer500ms = 0;
int FM = 0;

bool Route = 0;
bool running = false;

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

int v_gauche = vitesse_normale;
int v_droite = vitesse_normale;

void alignement(void)
{
  if((lineSensorValues[1] > THRESHOLD_HIGH) && !(lineSensorValues[2] > THRESHOLD_HIGH))
  {
    //Serial1.println("OUI1");
    v_droite = v_droite + diff_vitesse;
    v_gauche = 0;
    //v_gauche = v_gauche - diff_vitesse;
    
    motors.setSpeeds(v_gauche, v_droite);
  }
  else if(lineSensorValues[3] > THRESHOLD_HIGH && !(lineSensorValues[2] > THRESHOLD_HIGH))
  {
    //Serial1.println("OUI2");
    v_gauche = v_gauche + diff_vitesse;
    v_droite = 0;
    //v_droite = v_droite - diff_vitesse;
    motors.setSpeeds(v_gauche, v_droite);
  }
   else
  {
    //Serial1.println("OUI3");
    v_gauche = vitesse_normale;
    v_droite = vitesse_normale;
    motors.setSpeeds(v_gauche, v_droite);
  }
}

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
  if(CapCentralRead == 0 &&  (lineSensorValues[2] > 100))
  {
    CapCentralRead = 1;
    return true;
  }
  else if(CapCentralRead == 1 &&  (lineSensorValues[2] < 100))
  {
    CapCentralRead = 0;
    return false;
  }

  else
  {
    return false;
  }
  
}

void decoder() {
  if(!Route)
  {
  if(detectcode())
  {
    if(n<=7)
    {
      char1 = (char1 << 1) | (lineSensorValues[0] > 100);
      char2 = (char2 << 1) | (lineSensorValues[4] > 100);
      n++;
    }
    else if (n<16)
    {
      char3 = (char3 << 1) | (lineSensorValues[0] > 100);
      char4 = (char4 << 1) | (lineSensorValues[4] > 100);
      n++;
    }
  }
  }
  else if(Route)
  {
    if(detectcode())
    {
    //Serial1.println("JE RENTRE");
    if(n<=7)
    {
      char5 = (char5 << 1) | (lineSensorValues[0] > 100);
      char6 = (char6 << 1) | (lineSensorValues[4] > 100);
      n++;
    }
    else if (n<16)
    {
      char7 = (char7 << 1) | (lineSensorValues[0] > 100);
      char8 = (char8 << 1) | (lineSensorValues[4] > 100);
      n++;
    }
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
  motors.setSpeeds(v_gauche, v_droite);  // Avanza hacia adelante con velocidad 200
}

void setup() {
  initSystem();
}


void loop() {
  if(Serial1.available()){
    char receivedchar = Serial1.read();
    if (receivedchar == 'G')running = true;
    if (receivedchar == 'S')running = false;
  }
  
  if(running){
    // Obtener posición de la línea
    int16_t position = lineSensors.readLine(lineSensorValues);
    //currentState = State::DECODE;
    switch (currentState){
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
            else if ((millis() - timer1s) >= 1000) {
              Serial1.println("STRAIGHT");
                currentState = State::STRAIGHT;
            } 
            break;

        case State::DECODE: 
            alignement();
            //Serial1.print("N: "); Serial1.println(n);
            if(n<16)
            {
              Serial1.print("N: "); Serial1.println(n);
              decoder();
            }
            else
            {
              if(!Route){
                
                if(AllOnWhite())
                  {delay(700);
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
                  {delay(700);
                    Serial1.print(char5);
                    Serial1.print(char6);
                    Serial1.print(char7);
                    Serial1.println(char8);
                    Serial1.println("FOLLOW_LINE");
                    currentState = State::FOLLOW_LINE;
                    n=0;
                    Route = 0;
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
}
