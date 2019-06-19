// /*
//         Arduino Brushless Motor Control
//      by Dejan, https://howtomechatronics.com
// */
//
// #include <Servo.h>
//
// #define MIN_PULSE_LENGTH 1000
// #define MAX_PULSE_LENGTH 2000
// #define OUTPUT_PIN 8
//
// Servo esc;     // create servo object to control the ESC
// char data;
//
// void setup() {
//
//   Serial.begin(9600);
//   // Attach the ESC on pin 9
//   esc.attach(OUTPUT_PIN, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); // (pin, min pulse width, max pulse width in microseconds)
//
//   displayInstructions();
// }
//
// void loop() {
//   data = Serial.read() - '0';
//
//   switch(data){
//     case 0 :
//       Serial.println("Sending minimum throttle");
//       esc.writeMicroseconds(MIN_PULSE_LENGTH);
//       break;
//     case 1:
//       Serial.println("Sending maximum throttle");
//       esc.writeMicroseconds(MAX_PULSE_LENGTH);
//       break;
//     case 2:
//       Serial.print("Running test");
//       test();
//       break;
//   }
//
//   // delay(10*1000); // wait 10 seconds for min output check
//   // int maxPower=90; // can go upto 180
//   // maxPower = min(maxPower, 135);
//   // int accTime=30*1000;
//   // int accStepDelay=accTime/maxPower;
//
//   // //accelerate
//   // for(int powerValue=0; powerValue<maxPower; powerValue++){
//   //   ESC.write(powerValue);
//   //   delay(accStepDelay);
//   // }
//
//   // // deaccelarate
//   // for(int powerValue=maxPower-1; powerValue>=0; powerValue--){
//   //   ESC.write(powerValue);
//   //   delay(accStepDelay);
//   // }
// }
//
// void test(){
//   Serial.println("Revving up...");
//   REV_TILL=MIN_PULSE_LENGTH + (MAX_PULSE_LENGTH - MIN_PULSE_LENGTH)/2;
//   for (int i = MIN_PULSE_LENGTH; i < REV_TILL; i+=5)
//   {
//     Serial.print("Pulse length = ");
//     Serial.print(i);
//     esc.writeMicroseconds(i);
//     delay(200);
//   }
//   Serial.println("Stopping...");
//   esc.writeMicroseconds(MIN_PULSE_LENGTH);
// }
//
// void displayInstructions()
// {
//     Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
//     Serial.println("\t0 : Send min throttle");
//     Serial.println("\t1 : Send max throttle");
//     Serial.println("\t2 : Run test function\n");
// }
//
//
//
//
//
//
//
//
//
// #include <Wire.h>
// #include <Servo.h>
//
//
// Servo right_prop;
// Servo left_prop;
//
// /*MPU-6050 gives you 16 bits data so you have to create some 16int constants
//  * to store the data for accelerations and gyro*/
//
// int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
//
//
// float Acceleration_angle[2];
// float Gyro_angle[2];
// float Total_angle[2];
//
//
//
//
// float elapsedTime, time, timePrev;
// int i;
// float rad_to_deg = 180/3.141592654;
//
// float PID, pwmLeft, pwmRight, error, previous_error;
// float pid_p=0;
// float pid_i=0;
// float pid_d=0;
// /////////////////PID CONSTANTS/////////////////
// double kp=3.55;//3.55
// double ki=0.005;//0.003
// double kd=2.05;//2.05
// ///////////////////////////////////////////////
//
// double throttle=1300; //initial value of throttle to the motors
// float desired_angle = 0; //This is the angle in which we whant the
//                          //balance to stay steady
//
//
// void setup() {
//   Wire.begin(); //begin the wire comunication
//   Wire.beginTransmission(0x68);
//   Wire.write(0x6B);
//   Wire.write(0);
//   Wire.endTransmission(true);
//   Serial.begin(250000);
//   right_prop.attach(3); //attatch the right motor to pin 3
//   left_prop.attach(5);  //attatch the left motor to pin 5
//
//   time = millis(); //Start counting time in milliseconds
//   /*In order to start up the ESCs we have to send a min value
//    * of PWM to them before connecting the battery. Otherwise
//    * the ESCs won't start up or enter in the configure mode.
//    * The min value is 1000us and max is 2000us, REMEMBER!*/
//   left_prop.writeMicroseconds(1000);
//   right_prop.writeMicroseconds(1000);
//   delay(7000); /*Give some delay, 7s, to have time to connect
//                 *the propellers and let everything start up*/
// }//end of setup void
//
// void loop() {
//
// /////////////////////////////I M U/////////////////////////////////////
//     timePrev = time;  // the previous time is stored before the actual time read
//     time = millis();  // actual time read
//     elapsedTime = (time - timePrev) / 1000;
//
//   /*The tiemStep is the time that elapsed since the previous loop.
//    * This is the value that we will use in the formulas as "elapsedTime"
//    * in seconds. We work in ms so we haveto divide the value by 1000
//    to obtain seconds*/
//
//   /*Reed the values that the accelerometre gives.
//    * We know that the slave adress for this IMU is 0x68 in
//    * hexadecimal. For that in the RequestFrom and the
//    * begin functions we have to put this value.*/
//
//      Wire.beginTransmission(0x68);
//      Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
//      Wire.endTransmission(false);
//      Wire.requestFrom(0x68,6,true);
//
//    /*We have asked for the 0x3B register. The IMU will send a brust of register.
//     * The amount of register to read is specify in the requestFrom function.
//     * In this case we request 6 registers. Each value of acceleration is made out of
//     * two 8bits registers, low values and high values. For that we request the 6 of them
//     * and just make then sum of each pair. For that we shift to the left the high values
//     * register (<<) and make an or (|) operation to add the low values.*/
//
//      Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
//      Acc_rawY=Wire.read()<<8|Wire.read();
//      Acc_rawZ=Wire.read()<<8|Wire.read();
//
//
//     /*///This is the part where you need to calculate the angles using Euler equations///*/
//
//     /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw
//      * values that we have just read by 16384.0 because that is the value that the MPU6050
//      * datasheet gives us.*/
//     /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
//     * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
//     * to calculate this value in each loop we have done that just once before the setup void.
//     */
//
//     /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
//      *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
//      *  will calculate the rooth square.*/
//      /*---X---*/
//      Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
//      /*---Y---*/
//      Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
//
//    /*Now we read the Gyro data in the same way as the Acc data. The adress for the
//     * gyro data starts at 0x43. We can see this adresses if we look at the register map
//     * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for
//     * the Z axis (YAW).*/
//
//    Wire.beginTransmission(0x68);
//    Wire.write(0x43); //Gyro data first adress
//    Wire.endTransmission(false);
//    Wire.requestFrom(0x68,4,true); //Just 4 registers
//
//    Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
//    Gyr_rawY=Wire.read()<<8|Wire.read();
//
//    /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
//    the raw value by 131 because that's the value that the datasheet gives us*/
//
//    /*---X---*/
//    Gyro_angle[0] = Gyr_rawX/131.0;
//    /*---Y---*/
//    Gyro_angle[1] = Gyr_rawY/131.0;
//
//    /*Now in order to obtain degrees we have to multiply the degree/seconds
//    *value by the elapsedTime.*/
//    /*Finnaly we can apply the final filter where we add the acceleration
//    *part that afects the angles and ofcourse multiply by 0.98 */
//
//    /*---X axis angle---*/
//    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
//    /*---Y axis angle---*/
//    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
//
//    /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
//     //Serial.println(Total_angle[1]);
//
//
//
// /*///////////////////////////P I D///////////////////////////////////*/
// /*Remember that for the balance we will use just one axis. I've choose the x angle
// to implement the PID with. That means that the x axis of the IMU has to be paralel to
// the balance*/
//
// /*First calculate the error between the desired angle and
// *the real measured angle*/
// error = Total_angle[1] - desired_angle;
//
// /*Next the proportional value of the PID is just a proportional constant
// *multiplied by the error*/
//
// pid_p = kp*error;
//
// /*The integral part should only act if we are close to the
// desired position but we want to fine tune the error. That's
// why I've made a if operation for an error between -2 and 2 degree.
// To integrate we just sum the previous integral value with the
// error multiplied by  the integral constant. This will integrate (increase)
// the value each loop till we reach the 0 point*/
// if(-3 <error <3)
// {
//   pid_i = pid_i+(ki*error);
// }
//
// /*The last part is the derivate. The derivate acts upon the speed of the error.
// As we know the speed is the amount of error that produced in a certain amount of
// time divided by that time. For taht we will use a variable called previous_error.
// We substract that value from the actual error and divide all by the elapsed time.
// Finnaly we multiply the result by the derivate constant*/
//
// pid_d = kd*((error - previous_error)/elapsedTime);
//
// /*The final PID values is the sum of each of this 3 parts*/
// PID = pid_p + pid_i + pid_d;
//
// /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
// tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
// have a value of 2000us the maximum value taht we could sybstract is 1000 and when
// we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
// to reach the maximum 2000us*/
// if(PID < -1000)
// {
//   PID=-1000;
// }
// if(PID > 1000)
// {
//   PID=1000;
// }
//
// /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
// pwmLeft = throttle + PID;
// pwmRight = throttle - PID;
//
//
// /*Once again we map the PWM values to be sure that we won't pass the min
// and max values. Yes, we've already maped the PID values. But for example, for
// throttle value of 1300, if we sum the max PID value we would have 2300us and
// that will mess up the ESC.*/
// //Right
// if(pwmRight < 1000)
// {
//   pwmRight= 1000;
// }
// if(pwmRight > 2000)
// {
//   pwmRight=2000;
// }
// //Left
// if(pwmLeft < 1000)
// {
//   pwmLeft= 1000;
// }
// if(pwmLeft > 2000)
// {
//   pwmLeft=2000;
// }
//
// /*Finnaly using the servo function we create the PWM pulses with the calculated
// width for each pulse*/
// left_prop.writeMicroseconds(pwmLeft);
// right_prop.writeMicroseconds(pwmRight);
// previous_error = error; //Remember to store the previous error.
//
// }//end of loop void
//
//
//
//
//
//
// const bool escCalibrateMode = false;
//
// // class default I2C address is 0x68
// // specific I2C addresses may be passed as a parameter here
// // AD0 low = 0x68 (default for InvenSense evaluation board)
// // AD0 high = 0x69
// MPU6050 mpu;
// //MPU6050 mpu(0x69); // <-- use for AD0 high
//
// bool dmpReady;
// uint8_t devStatus;
// uint8_t packetSize;
// uint16_t fifoCount;
//
// uint8_t execState;
//
// uint16_t calibrationStartTime;
// const uint16_t calibrationTime = 20000;
//
//
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
//
// Quaternion q;
// float ypr[3];
// float ypr_offset[3];
// bool calibrated;
//
// VectorFloat gravity;
// uint8_t fifoBuffer[64];
//
// Servo rProp;
//
// void calibrateProps(){
//   Serial.println("calibrating esc...");
//   Serial.println("max throttle");
//   setRev(MAX_PULSE_LENGTH);
//   delay(1500);
//   Serial.println("min throttle");
//   setRev(MIN_PULSE_LENGTH);
//   delay(2000);
//   Serial.println("calibration done");
//
// }
//
// void caliberateGround(){
//   // the values take 20 s to stabilize
//   while(millis() - calibrationStartTime < calibrationTime)
//     getYPR();
//
//   for(int i=0; i<3; i++){
//     ypr_offset[i] = ypr[i];
//   }
//   Serial.println("ground offset values: ");
//   printYPR();
//   execState = STATE_READY;
//
//
//
//   // VectorInt16 orientation;
//   // mpu.dmpGetAccel(&orientation, fifoBuffer);
//   //
//   // Serial.print("initial offsets: \t");
//   // Serial.print(orientation.x);
//   // Serial.print("\t");
//   // Serial.print(orientation.y);
//   // Serial.print("\t");
//   // Serial.println(orientation.z);
//   //
//   // mpu.setXAccelOffset(orientation.x);
//   // mpu.setYAccelOffset(orientation.y);
//   // mpu.setZAccelOffset(orientation.z);
// }
//
// void setupProps(){
//   rProp.attach(RIGHT_PROP_OUTPUT_PIN, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
//   if(escCalibrateMode)
//     setRev(MAX_PULSE_LENGTH);
//   else
//     stopRev();
// }
//
//
// void setup() {
//   execState = STATE_SETUP;
//   // join I2C bus (I2Cdev library doesn't do this automatically)
//   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//   Wire.begin();
//   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//   Fastwire::setup(400, true);
//   #endif
//
//   Serial.begin(9600);
//   while (!Serial);
//
//   Serial.println(F("Initializing I2C devices..."));
//   mpu.initialize();
//
//   // verify connection
//   Serial.println(F("Testing device connections..."));
//   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//
//   //setup props
//   setupProps();
//
//   // wait for ready
//   Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//   execState = STATE_WAIT_INTERRUPT;
//   // while (Serial.available() && Serial.read()); // empty buffer
//   // while (!Serial.available());                 // wait for data
//   // while (Serial.available() && Serial.read()); // empty buffer again
//
//   if(escCalibrateMode)
//     calibrateProps();
//   pulseRev();
//   stopRev();
//
//   // load and configure the DMP
//   Serial.println(F("Initializing DMP..."));
//   devStatus = mpu.dmpInitialize();
//
//   // supply your own gyro offsets here, scaled for min sensitivity
//   mpu.setXGyroOffset(220);
//   mpu.setYGyroOffset(76);
//   mpu.setZGyroOffset(-85);
//   mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
//
//   // make sure it worked (returns 0 if so)
//   if (devStatus != 0) {
//     // ERROR!
//     // 1 = initial memory load failed
//     // 2 = DMP configuration updates failed
//     // (if it's going to break, usually the code will be 1)
//     Serial.print(F("DMP Initialization failed (code "));
//     Serial.print(devStatus);
//     Serial.println(F(")"));
//     return;
//   }
//
//   // turn on the DMP, now that it's ready
//   Serial.println(F("Enabling DMP..."));
//   mpu.setDMPEnabled(true);
//
//   // get expected DMP packet size for later comparison
//   packetSize = mpu.dmpGetFIFOPacketSize();
//   dmpReady = true;
//
//   execState = STATE_CALIBRATING;
//   Serial.println("Calibrating...");
//   calibrationStartTime = millis();
//   caliberateGround();
// }
//
//
//
// void getYPR(){
//   waitBuffer();
//   mpu.dmpGetQuaternion(&q, fifoBuffer);
//   mpu.dmpGetGravity(&gravity, &q);
//   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//   if(execState==STATE_READY)
//     for (int i = 0; i < 3; i++)
//       ypr[i]-=ypr_offset[i]; // TODO: this logic breaks for ceiling values -180 to 180
// }
//
// void printYPR(){
//   #ifdef OUTPUT_READABLE_YAWPITCHROLL
//   // display Euler angles in degrees
//   Serial.print("ypr\t");
//   Serial.print(ypr[0] * 180/M_PI);
//   Serial.print("\t");
//   Serial.print(ypr[1] * 180/M_PI);
//   Serial.print("\t");
//   Serial.println(ypr[2] * 180/M_PI);
//   #endif
//
//
//   #ifdef OUTPUT_READABLE_ACCELGYRO
//   // read raw accel/gyro measurements from device
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//   // display tab-separated accel/gyro x/y/z values
//   Serial.print("a/g:\t");
//   Serial.print(ax); Serial.print("\t");
//   Serial.print(ay); Serial.print("\t");
//   Serial.print(az); Serial.print("\t");
//   Serial.print(gx); Serial.print("\t");
//   Serial.print(gy); Serial.print("\t");
//   Serial.println(gz);
//   #endif
// }
//
// void loop() {
//   switch (execState) {
//     case STATE_READY:
//       getYPR();
//       printYPR();
//       break;
//     default:
//       Serial.println("UNK STATE");
//       break;
//   }
// }
