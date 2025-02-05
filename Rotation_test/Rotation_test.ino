#include <Wire.h>
#include <Zumo32U4.h>

const int32_t turnSpeed = 150; // Vitesse de rotation
const int32_t ticksPerTurn = 2490; // Nombre de ticks pour 360°
const int32_t ticks45 = ticksPerTurn * 45 / 360;  // 45°
const int32_t ticks90 = ticksPerTurn * 90 / 360;  // 90°
const int32_t ticks120 = ticksPerTurn * 120 / 360; // 135°

Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

void setup()
{
    Serial1.begin(9600);
    display.clear();
    display.print(F("Press A B C"));
}

void turnAngle(int16_t targetTicks)
{
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();

    while (abs(encoders.getCountsRight()) < targetTicks)
    {
        Serial1.println(encoders.getCountsRight());
        motors.setSpeeds(-turnSpeed, turnSpeed); // Rotation sur place
    }

    motors.setSpeeds(0, 0); // Stop rotation
}

void loop()
{
    if (Serial1.read() == 'A')
    {
      Serial1.println(ticks45);
        turnAngle(ticks45);
    }

    if (Serial1.read() == 'B')
    {
        Serial1.println(ticks90);
        turnAngle(ticks90);
    }

    if (Serial1.read() == 'C')
    {
        Serial1.println(ticks120);
        turnAngle(ticks120);
    }
}
