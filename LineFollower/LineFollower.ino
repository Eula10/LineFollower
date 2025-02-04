#include <Wire.h>
#include <Zumo32U4.h>

#define maxSpeed 400
#define minSpeed 41
#define NUM_SENSORS 5

#define THRESHOLD_HIGH 800  // Umbral alto
#define THRESHOLD_LOW 200    // Umbral bajo

// Este es el límite de velocidad para los motores
const uint16_t limSpeed = 2 * minSpeed;

bool useEmitters = true;

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;

enum Estado {
    SIGUE_LINEA,
    ZONA_NEGRA,
    ZONA_BLANCA
};

Estado estadoActual = SIGUE_LINEA;

// Definir constantes PID para fácil modificación
float Kp = 0.5;  // Proporcional
float Ki = 0.0;   // Integral (ajustar si es necesario)
float Kd = 3.0;   // Derivativo

int16_t integral = 0;  // Acumulador del término integral
int16_t lastError = 0; // Último error para el cálculo derivativo

char buffer[10];

static uint16_t lastSampleTime = 0;

unsigned int lineSensorValues[NUM_SENSORS];

void calibrateSensors() {
   //Espera 1 segundo y luego calibra los sensores girando
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

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  lineSensors.initFiveSensors();

  // Espera a que se presione el botón C antes de iniciar
  //buttonC.waitForButton();

  calibrateSensors();

  // Inicializar integral en 0
  integral = 0;
  lastSampleTime = 0;

  Serial1.write("START\n");
}

void loop() {
  // Obtener la posición de la línea
  int16_t position = lineSensors.readLine(lineSensorValues);
//  sprintf(buffer, "%d", position);  
//  Serial1.write(buffer);
//  Serial1.println("\n");

  // Evaluar cambio de estado
    switch (estadoActual) {
        case SIGUE_LINEA:
            Serial1.println("Follow line\n");
            followLine(position);
            if (todosSobreNegro()) {
                estadoActual = ZONA_NEGRA;
            }
            break;

        case ZONA_NEGRA:
            Serial1.println("Black zone\n");
            motors.setSpeeds(limSpeed, limSpeed);  // Avanzar recto
            if (todosSobreBlanco()) {
                estadoActual = ZONA_BLANCA;
            }
            break;

        case ZONA_BLANCA:
            Serial1.println("White zone\n");
            stayInWhiteZone();
            break;
    }
//    delay(500);
}


void followLine(int16_t position) {
  // Calcular el error (desviación del centro)
  int16_t error = position - 2000;

  // Calcular el término integral (ajustar Ki antes de usarlo)
  integral += error;

  // Calcular la corrección PID
  int16_t speedDifference = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));

  lastError = error; // Guardar el error actual para el próximo cálculo derivativo

  // Calcular las velocidades de los motores
  int16_t leftSpeed = (int16_t)limSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)limSpeed - speedDifference;

  // Restringir las velocidades para evitar valores fuera de rango
  leftSpeed = constrain(leftSpeed, 0, (int16_t)limSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)limSpeed);

  // Aplicar las velocidades a los motores
  motors.setSpeeds(leftSpeed, rightSpeed);
}

// Verificar si todos los sensores detectan negro (> 1000)
bool todosSobreNegro() {
//  char buffer[80];
//  sprintf(buffer, "%4d %4d %4d %4d %4d %c\n",
//    lineSensorValues[0],
//    lineSensorValues[1],
//    lineSensorValues[2],
//    lineSensorValues[3],
//    lineSensorValues[4],
//    useEmitters ? 'E' : 'e'
//  );
//  Serial1.print(buffer);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] < THRESHOLD_HIGH) {
            return false;
        }
    }
    return true;
}

// Verificar si todos los sensores detectan blanco (< 200)
bool todosSobreBlanco() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (lineSensorValues[i] > THRESHOLD_LOW) {
            return false;
        }
    }
    return true;
}

void stayInWhiteZone() {
    Serial1.println("Zona Blanca");
    motors.setSpeeds(0, 0);
}
