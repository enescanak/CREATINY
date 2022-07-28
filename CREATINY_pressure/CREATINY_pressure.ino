#define SAMPLE_RATE 10
#define CONSOLE_DELAY 100
#define COMM_DELAY 50
#define REVERSE 1
#define FORWARD 0
#define CONSOLE_ON 1
#define CONSOLE_OFF 0

#include "SparkFunLSM6DS3.h"
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>
#include "MS5837.h"
#include "QuickPID.h"
//#include "esc.h"
#include "SerialTransfer.h"
#include "Servo.h"

unsigned long microsPerReading, microsPrevious;
unsigned long consolePerReading, consoleMilisPrevious, consoleMillisNow;
unsigned long commPerReading, commMilisPrevious, commMillisNow;
unsigned long IMUMillisNow;
float SetpointHeading = 0, InputHeading = 0, OutputHeading = 0;
float Kp = 1, Ki = 0, Kd = 0;
float roll, pitch, heading, headingOffset;
float pressure, temp, depth, altitude;
int valueJoyStick_X_1 = 1500, valueJoyStick_Y_1 = 1500, valueJoyStick_X_2 = 1500, valueJoyStick_Y_2 = 1500;
int16_t Dist1, Dist2, Dist3;
bool IMUflag = false;


LSM6DS3 lsm6ds3(I2C_MODE, 0x6A);
QMC5883LCompass qmc5883;
MS5837 ms5837;
Madgwick filter;
Servo ESc1, ESc2, ESc3, ESc4, ESc5, ESc6;

QuickPID pid(&InputHeading, &OutputHeading, &SetpointHeading);

SerialTransfer myTransfer;

void setup() {

  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();

  ESc1.attach(23, 1000, 2000);
  ESc2.attach(19, 1000, 2000);
  ESc3.attach(20, 1000, 2000);
  ESc4.attach(24, 1000, 2000);
  ESc5.attach(21, 1000, 2000);
  ESc6.attach(22, 1000, 2000);


  Serial.begin(115200);
  myTransfer.begin(Serial);
  ms5837.init();
  lsm6ds3.begin();
  qmc5883.init();
  lsm6ds3.calcGyroOffsets(3000);
  filter.begin(SAMPLE_RATE);
  armESC(3200);

  pinMode(25,OUTPUT);

  SetpointHeading = 0;
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetOutputLimits(-500, 500);
  pid.SetMode(pid.Control::automatic);

  ms5837.setModel(MS5837::MS5837_30BA);
  ms5837.setFluidDensity(997);
  microsPerReading = 1000000 / SAMPLE_RATE;
  microsPrevious = micros();

  consolePerReading = CONSOLE_DELAY;
  consoleMilisPrevious = millis();
  commPerReading = COMM_DELAY;
  commMilisPrevious = millis();
}

void loop() {

  IMUSensorValue(CONSOLE_OFF, false);
 
//  pid.Compute();


  Comm();
  //MotorDrive(CONSOLE_OFF, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD);
}

void IMUSensorValue(bool console, bool filterWithMag) {

  float gx, gy, gz, ax, ay, az, mx, my, mz;
  unsigned long microsNow;
  qmc5883.read();

  gx = lsm6ds3.readFloatGyroX();
  gy = lsm6ds3.readFloatGyroY();
  gz = lsm6ds3.readFloatGyroZ();
  ax = lsm6ds3.readFloatAccelX();
  ay = lsm6ds3.readFloatAccelY();
  az = lsm6ds3.readFloatAccelZ();
  mx = qmc5883.getX();
  my = qmc5883.getY();
  mz = qmc5883.getZ();

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    (filterWithMag) ? filter.update(gx, gy, gz, ax, ay, az, mx, my, mz) : filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    
    IMUMillisNow = millis();
    if(IMUMillisNow >= 10000 && IMUflag == false){
    headingOffset = heading;
    IMUflag = true;
    }
    if(IMUflag){
    digitalWrite(25,HIGH);
    InputHeading = heading - headingOffset;
    //Serial.print("InputHeading: ");
    //Serial.print(InputHeading);
    }
    //Serial.print(" OutputHeading: ");
    //Serial.println((int)OutputHeading);
    microsPrevious = microsPrevious + microsPerReading;
  }
  if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {

      Serial.print("heading: ");
      Serial.print(heading);
      Serial.print(" pitch: ");
      Serial.print(pitch);
      Serial.print(" roll: ");
      Serial.print(roll);
      Serial.print(" gyroX: ");
      Serial.print(gx);
      Serial.print(" gyroY: ");
      Serial.print(gy);
      Serial.print(" gyroZ: ");
      Serial.println(gz);
      //Serial.print(" magX: ");
      //Serial.print(mx);
      //Serial.print(" magY: ");
      //Serial.print(my);
      //Serial.print(" magZ: ");
      //Serial.println(mz);

      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
    }
  }
}

void MotorDrive(bool console, bool RiseM1Direct, bool RiseM2Direct, bool FrontRightDirect, bool FrontLeftDirect, bool BackRightDirect, bool BackLeftDirect) {
  int RiseM1, RiseM2, FrontRight, FrontLeft, BackRight, BackLeft;

  RiseM1 = valueJoyStick_X_1;  // sol orta
  RiseM2 = valueJoyStick_X_1;  // sag orta
  FrontRight = 1500 + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  FrontLeft = 1500 - (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  BackRight = 1500 + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  BackLeft = 1500 - (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);

  if (RiseM1Direct) RiseM1 -= 3000;
  if (RiseM2Direct) RiseM2 -= 3000;
  if (FrontRightDirect) FrontRight -= 3000;
  if (FrontLeftDirect) FrontLeft -= 3000;
  if (BackRightDirect) BackRight -= 3000;
  if (BackLeftDirect) BackLeft -= 3000;

  constrain(RiseM1, 1000, 2000);
  constrain(RiseM2, 1000, 2000);
  constrain(FrontRight, 1000, 2000);
  constrain(FrontLeft, 1000, 2000);
  constrain(BackRight, 1000, 2000);
  constrain(BackLeft, 1000, 2000);

  ESc1.writeMicroseconds(RiseM1);
  ESc2.writeMicroseconds(RiseM2);
  ESc3.writeMicroseconds(FrontRight);
  ESc4.writeMicroseconds(FrontLeft);
  ESc5.writeMicroseconds(BackRight);
  ESc6.writeMicroseconds(BackLeft);

  if (console) {
    consoleMillisNow = millis();
    if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {

      Serial.print("MotorValue: ");
      Serial.print(RiseM1);
      Serial.print(" ");
      Serial.print(RiseM2);
      Serial.print(" ");
      Serial.print(FrontRight);
      Serial.print(" ");
      Serial.print(FrontLeft);
      Serial.print(" ");
      Serial.print(BackRight);
      Serial.print(" ");
      Serial.println(BackLeft);

      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
    }
  }
}


void Comm() {
  struct STRUCT {
    int32_t accX = 0;
    int32_t accY = 0;
    int32_t accZ = 0;
    int32_t gyroX = 0;
    int32_t gyroY = 0;
    int32_t gyroZ = 0;
    int32_t pressure = 0;
    int32_t batteryVoltage = 0;
    int32_t batteryAmp = 0;
    int32_t waterTemp = 0;
    int32_t internalTemp = 0;
   } rovDataTx;

  struct STRUCT1 {
    int32_t button;
    int32_t leftTrigger;
    int32_t rightTrigger;
    int32_t leftThumbX;
    int32_t leftThumbY;
    int32_t rightThumbX;
    int32_t rightThumbY;
  } rovDataRx;

  rovDataTx.accX = 0;
  rovDataTx.accY = 0;
  rovDataTx.accZ = 0; 
  rovDataTx.gyroX = int(pitch);
  rovDataTx.gyroY = int(roll);
  rovDataTx.gyroZ = int(heading);
  rovDataTx.pressure = 0;
  rovDataTx.batteryVoltage = 0;
  rovDataTx.batteryAmp = 0;
  rovDataTx.waterTemp = 0;
  rovDataTx.internalTemp = 0;
   

  if (myTransfer.available()) {
    commMillisNow = millis();
    if (commMillisNow - commMilisPrevious >= commPerReading) {
      uint16_t recSize = 0;
      uint16_t sendSize = 0;

      recSize = myTransfer.rxObj(rovDataRx, recSize);

      sendSize = myTransfer.txObj(rovDataTx, sendSize);

      myTransfer.sendData(sendSize);
      commMilisPrevious = commMilisPrevious + commPerReading;
    }
    valueJoyStick_X_2 = rovDataRx.leftThumbX;
    valueJoyStick_X_1 = rovDataRx.leftThumbY;
    valueJoyStick_Y_2 = rovDataRx.rightThumbX;
    valueJoyStick_Y_1 = rovDataRx.rightThumbY;
  } 
}

  void pres_sensor_values(bool console) {

    ms5837.read();
    pressure = ms5837.pressure();
    temp = ms5837.temperature();
    depth = ms5837.depth();
    altitude = ms5837.altitude();
    if (console) {
      consoleMillisNow = millis();
      if (consoleMillisNow - consoleMilisPrevious >= consolePerReading) {

        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" mbar");

        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println(" deg C");

        Serial.print("Depth: ");
        Serial.print(depth);
        Serial.println(" m");

        Serial.print("Altitude: ");
        Serial.print(altitude);


        consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
      }
    }
  }

  void armESC(int delayESC) {

    ESc1.writeMicroseconds(1500);
    ESc2.writeMicroseconds(1500);
    ESc3.writeMicroseconds(1500);
    ESc4.writeMicroseconds(1500);
    ESc5.writeMicroseconds(1500);
    ESc6.writeMicroseconds(1500);

    delay(delayESC);
  }
