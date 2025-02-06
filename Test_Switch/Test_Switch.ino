#include <Wire.h>
#include <Zumo32U4.h>

enum class State : uint8_t {
    FOLLOW_LINE,
    BLACK_ZONE,
    WHITE_ZONE,
    TURN_RIGHT,
    TURN_LEFT,
    COMBAT_MODE
};

volatile State currentState = State::FOLLOW_LINE;

bool running = false;

void setup() {
    Serial1.begin(38400);

}

void loop() {
  if (!running) {
    Serial1.print("Stop");
        return;
  }

  Serial1.print("currentState = ");
  Serial1.println(static_cast<uint8_t>(currentState));
  switch (currentState) {
    case State::FOLLOW_LINE:
      currentState = State::BLACK_ZONE;
      break;
      
    case State::BLACK_ZONE:
      currentState = State::WHITE_ZONE;
      break;
      
    case State::WHITE_ZONE:
      currentState = State::TURN_RIGHT;
      break;

    case State::TURN_LEFT:
    case State::TURN_RIGHT:
      currentState = State::WHITE_ZONE;
      break;

    case State::COMBAT_MODE:
      Serial1.println("Combat Mode On");
      Serial1.print("Edge = ");
      break;

    default:
      Serial1.println("indefinido");
      break;
  }
  delay(1000);
}

  // Esta función se ejecuta automáticamente cuando hay datos en Serial1
void serialEvent1() {
    while (Serial1.available()) {  
        char receivedChar = Serial1.read();  
        if (receivedChar == 'S') running = true;
        if (receivedChar == 'P') running = false;
        if (receivedChar == 'B');
        if (receivedChar == 'C') currentState = State::COMBAT_MODE;
        if (receivedChar == 'D') currentState = State::WHITE_ZONE;
        if (receivedChar == 'N');
        if (receivedChar == '1');
        if (receivedChar == '2');
        if (receivedChar == '3');
        if (receivedChar == '4');
        Serial1.print("Recibido: ");
        Serial1.println(receivedChar);
        Serial1.print("Nuevo estado: ");
        Serial1.println(static_cast<uint8_t>(currentState));
    }
}
