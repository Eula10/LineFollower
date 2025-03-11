const int MIN_SPEED = 60;
int LIM_SPEED = 2 * MIN_SPEED; //Min speed = 41, Max speed = 400
const int NUM_SENSORS = 5;
const int REVERSE_DURATION = 500;

const int THRESHOLD_HIGH = 800;
const int THRESHOLD_LOW = 200;

const int32_t ticksPerTurn = 2490; // Nombre de ticks pour 360°
const int angles[] = {45, 90, 120}; // Angle list
int angleIndex = 0;  // Índice para iterar entre los ángulos
int commandAngle = 4;

int distance = 0;
bool measure = true;
float Distance = 0.0;

int direction = 1;  // 1 para giro a la derecha, -1 para giro a la izquierda

bool useEmitters = true;
bool running = false;

unsigned long startTime;
unsigned long endTime;

enum class State : uint8_t {
    FOLLOW_LINE,
    AVANCER_1s,
    DECODE,
    STRAIGHT,
    AVANCER_500ms,
    STOP
};

State currentState = State::FOLLOW_LINE;

// A sensors reading must be greater than or equal to this
// threshold in order for the program to consider that sensor as
// seeing an object.
const uint8_t sensorThreshold = 1;

// The maximum speed to drive the motors while turning. 
const uint16_t turnSpeedMax = 400;

// The minimum speed to drive the motors while turning. 
const uint16_t turnSpeedMin = 100;

// The amount to decrease the motor speed by during each cycle
// when an object is seen.
const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

#define LEFT 0
#define RIGHT 1

// Stores the last indication from the sensors about what
// direction to turn to face the object.  
bool senseDir = RIGHT;

uint16_t turnSpeed = turnSpeedMax;

// Define PID constants for easy modification
float Kp = 0.25;  // Proportional
float Ki = 0.0;   // Integral (adjust if necessary)
float Kd = 6.0;   // Derivative

int16_t integral = 0;  // Accumulator for the integral term
int16_t lastError = 0; // Last error for derivative calculation

char buffer[10];

static uint16_t lastSampleTime = 0;

unsigned int lineSensorValues[NUM_SENSORS];
