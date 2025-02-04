#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
bool running = false;  

void setup() {
    Serial1.begin(9600);  
}

void loop() {
    if (running) {
        motors.setSpeeds(100, 100);  
    } else {
        motors.setSpeeds(0, 0);  
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
