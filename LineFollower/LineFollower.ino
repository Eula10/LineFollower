/* This example uses the line sensors on the Zumo 32U4 to follow
a black line on a white background, using a PID-based algorithm.
It works decently on courses with smooth, 6" radius curves and
has been tested with Zumos using 75:1 HP motors.  Modifications
might be required for it to work well on different courses or
with different motors.

This demo requires a Zumo 32U4 Front Sensor Array to be
connected, and jumpers on the front sensor array must be
installed in order to connect pin 4 to DN4 and pin 20 to DN2. */

#include <Wire.h>
#include <Zumo32U4.h>

// This is the maximum speed the motors will be allowed to turn.
const uint16_t maxSpeed = 200;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonC buttonC;

// Definir constantes PID para fácil modificación
float Kp = 0.25;  // Proporcional
float Ki = 0.0;   // Integral (ajustar si es necesario)
float Kd = 6.0;   // Derivativo

int16_t integral = 0;  // Acumulador del término integral
int16_t lastError = 0; // Último error para el cálculo derivativo

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

void calibrateSensors() {
  // Espera 1 segundo y luego calibra los sensores girando
  delay(1000);
  for (uint16_t i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-200, 200);
    } else {
      motors.setSpeeds(200, -200);
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void setup() {
  Serial1.begin(9600);
  lineSensors.initFiveSensors();

  // Espera a que se presione el botón C antes de iniciar
  buttonC.waitForButton();

  calibrateSensors();

  // Inicializar integral en 0
  integral = 0;
}

void loop() {
  // Obtener la posición de la línea
  int16_t position = lineSensors.readLine(lineSensorValues);
  //Serial1.println(position);  // Enviar la posición por serial

  // Calcular el error (desviación del centro)
  int16_t error = position - 2000;

  // Calcular el término integral (ajustar Ki antes de usarlo)
  integral += error;

  // Calcular la corrección PID
  int16_t speedDifference = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));

  lastError = error; // Guardar el error actual para el próximo cálculo derivativo

  // Calcular las velocidades de los motores
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Restringir las velocidades para evitar valores fuera de rango
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  // Aplicar las velocidades a los motores
  motors.setSpeeds(leftSpeed, rightSpeed);
}
