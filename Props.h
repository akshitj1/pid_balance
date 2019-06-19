
#include "Servo.h"
#include "MathUtil.h"

class Props{
public:
  static const int kMaxThrottle = 250;
private:
  static const int kMinPulseLength=1000, kMaxPulseLength=2000, kPulseThrottle=100, kWarmThrottle = 50;
  const uint8_t kLeftPropOutPin, kRightPropOutPin;
  Servo lProp, rProp;

  void setRev(int revPulse){
    setRev(revPulse, lProp);
    setRev(revPulse, rProp);
  }

  void setRev(int revPulse, Servo &prop){
    prop.writeMicroseconds(revPulse);
  }


public:
  Props(const uint8_t leftPropOutPin,const uint8_t rightPropOutPin): kLeftPropOutPin(leftPropOutPin), kRightPropOutPin(rightPropOutPin){
    lProp.attach(kLeftPropOutPin, kMinPulseLength, kMaxPulseLength);
    rProp.attach(kRightPropOutPin, kMinPulseLength, kMaxPulseLength);
    stop();
  }

  void couple(int throttle){
    throttle = sgn(throttle) * min(kMaxThrottle, abs(throttle));
    static Servo* throttleProp;
    static uint32_t lPulse, rPulse;
    lPulse = kMinPulseLength + kWarmThrottle + ( sgn(throttle) > 0 ? abs(throttle) : 0);
    rPulse = kMinPulseLength + kWarmThrottle + ( sgn(throttle) < 0 ? abs(throttle) : 0);

    setRev(lPulse, lProp);
    setRev(rPulse, rProp);
  }


  void full(){
    setRev(kMaxPulseLength);
  }

  void stop(){
    setRev(kMinPulseLength);
  }

  void pulse(){
    setRev(kMinPulseLength +  kPulseThrottle);
    delay(10000);
  }

  void soft(){
    int REV_TILL = kMinPulseLength + 250;
    for (int i = kMinPulseLength; i < REV_TILL; i += 5)
    {
      Serial.print("Pulse length = ");
      Serial.println(i);
      setRev(i);
      delay(500);
    }
  }
};
