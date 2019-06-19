#include "BalancingStick.h"

const float Kp=500, Ki=3, Kd=0;//2.005*60;//Kp=3.55.=,Ki=0.005, Kd=2.005;
const uint8_t kLeftPropOutPin = 2, kRightPropOutPin = 3;
BalancingStick* balancingStick;

void waitSignal(){

  Serial.println("Input to begin...");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
}


void setup(){
  Serial.begin(9600);
  while (!Serial);

  Props props(kLeftPropOutPin, kRightPropOutPin);
  OrientationSensor oSensor;
  BalancingStick _balancingStick(oSensor, props, Kp, Ki, Kd);
  balancingStick = &_balancingStick;
  Serial.println("Setup complete...");

  waitSignal();
}


void loop()
{
  balancingStick->balance();
  delay(100);
}
