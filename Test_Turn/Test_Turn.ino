#include <Zumo32U4.h>

#define TICKS_PER_REVOLUTION 909.72  // Pulsos por revolución
#define WHEEL_RADIUS 2.0            // Radio de la rueda en cm (2 cm)
#define GEAR_RATIO 1.0              // Relación de engranaje (si la existe, ajusta si es necesario)

const float DISTANCE_PER_PULSE = (3.14159 * WHEEL_RADIUS * 2) / TICKS_PER_REVOLUTION;  // Distancia por pulso en cm
const int TARGET_DEGREES = 90;  // Giro de 90 grados

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

void setup() {
  Serial.begin(38400);
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void loop() {
  // Calcular la distancia que cada rueda debe recorrer para girar 90 grados sobre el eje
  float distanceToTravel = 7.854;  // Distancia a recorrer (7.854 cm para 90 grados)
  
  // Calcular cuántos pulsos por rueda se necesitan para recorrer esa distancia
  int targetTicks = distanceToTravel / DISTANCE_PER_PULSE;
   targetTicks = 700;
  
  // Rotar el robot en sentido horario (la rueda derecha avanza y la rueda izquierda retrocede)
  motors.setSpeeds(100, -100);  // Rueda izquierda va hacia atrás y derecha hacia adelante
  
  while (abs(encoders.getCountsLeft()) < targetTicks && abs(encoders.getCountsRight()) < targetTicks) {
    // Espera hasta que ambos encoders alcancen los pulsos necesarios para completar el giro
  }
  
  motors.setSpeeds(0, 0);  // Detiene el robot
  Serial.println(encoders.getCountsLeft());
  Serial.println(encoders.getCountsRight());
  Serial.println("Giro de 90 grados completado");

  // Espera un momento antes de comenzar otra acción
  delay(2000);  
}
