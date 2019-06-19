

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

using namespace std;

class Orientation{
public:
  float yaw=0, pitch=0, roll=0;
  Orientation(){};

  Orientation(const float ypr[3]) : yaw(ypr[0]), pitch(ypr[1]), roll(ypr[2]) {}

  void update(const float ypr[3]){
    yaw = ypr[0];
    pitch = ypr[1];
    roll = ypr[2];
  }

  void offset(const Orientation& ground){
    // TODO: this logic breaks for ceiling values -180 to 180
    yaw-=ground.yaw;
    pitch-=ground.pitch;
    roll-=ground.roll;
  }

  void print(){
    Serial.print(yaw * 180 / M_PI);
    Serial.print("\t");
    Serial.print(pitch * 180 / M_PI);
    Serial.print("\t");
    Serial.print(roll * 180 / M_PI);
    Serial.print("\n");
  }
};

class OrientationSensor{
private:
  MPU6050 mpu;

  const int kCalibrationTime = 20*1000;
  //todo: make this const
  int kPacketSize;
  uint8_t fifoBuffer[64];

  Orientation orientation, groundOffset;

  void waitBuffer(){
    if ((mpu.getFIFOCount() >= 1024)) {//_BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT) ||
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }

    // wait for buffer to fill atleast kPacketSize
    while (mpu.getFIFOCount() < kPacketSize);

    // get latest packet
    static uint8_t expiredPackets;
    expiredPackets=0;
    while(mpu.getFIFOCount() > kPacketSize){
      mpu.getFIFOBytes(fifoBuffer, kPacketSize);
      expiredPackets++;
    }

    if(expiredPackets>1){
      Serial.print("# expired packets: ");
      Serial.println(expiredPackets-1);
      mpu.resetFIFO();
    }
  }

  void getGroundOffsets(Orientation &groundOffset){
    Serial.println("Calibrating...");
    uint32_t calibrationStartTime = millis();

    // the values take 20 s to stabilize
    while(millis() - calibrationStartTime < kCalibrationTime)
      getOrientation(groundOffset);
    Serial.println("ground offset values: ");
    groundOffset.print();
  }

  void getOrientation(Orientation &orientation){
    waitBuffer();

    static Quaternion q;
    static VectorFloat gravity;
    static float ypr[3];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    orientation.update(ypr);
  }

public:

  OrientationSensor(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    int devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus != 0) {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      return;
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    kPacketSize = mpu.dmpGetFIFOPacketSize();

    getGroundOffsets(groundOffset);
  }

  Orientation& getCalibratedOrientation(){
    getOrientation(orientation);
    //orientation.print();
    orientation.offset(groundOffset);
    return orientation;
  }
};
