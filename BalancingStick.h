
#include "OrientationSensor.h"
#include "Props.h"
#include "PID_v1.h"


class BalancingStick{
private:
  PID pid;
  const double kTargetRoll = 0;
  Props &props;
  OrientationSensor &oSensor;
  double currentRoll, controlThrottle=0;

public:
  BalancingStick(OrientationSensor &oSensor, Props &props, float Kp, float Ki, float Kd):
  oSensor(oSensor),
  props(props),
  pid(&currentRoll, &controlThrottle, &kTargetRoll, (double)Kp, (double)Ki, (double)Kd, DIRECT){
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-1*Props::kMaxThrottle, Props::kMaxThrottle);
    pid.SetSampleTime(1000/20);
  }

  void balance(){
    currentRoll = oSensor.getCalibratedOrientation().roll;
    pid.Compute();

    Serial.print(currentRoll*180/M_PI);
    Serial.print("\t:\t");
    Serial.println(controlThrottle);
    props.couple(controlThrottle);
  }
};
