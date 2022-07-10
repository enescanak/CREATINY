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
#include "esc.h"
#include <SoftwareSerial.h>
#include <TFLI2C.h>
#include "SerialTransfer.h"

unsigned long microsPerReading, microsPrevious;
unsigned long consolePerReading, consoleMilisPrevious, consoleMillisNow;
unsigned long commPerReading, commMilisPrevious, commMillisNow;
float Setpoint, Input, Output;
float Kp = 2, Ki = 5, Kd = 1;
float roll, pitch, heading, headingOffset;
float pressure, temp, depth, altitude;
int valueJoyStick_X_1, valueJoyStick_Y_1, valueJoyStick_X_2, valueJoyStick_Y_2;
int16_t Dist1, Dist2, Dist3;



LSM6DS3 lsm6ds3(I2C_MODE, 0x6A);
QMC5883LCompass qmc5883;
MS5837 ms5837;
Madgwick filter;
ESC escControl(23, 20, 21, 22, 19, 24);
QuickPID pid(&Input, &Output, &Setpoint);
TFLI2C tflI2C;
SoftwareSerial luna1(6, 7);
SoftwareSerial luna3(4, 5);
SerialTransfer myTransfer;

void setup() {

  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();

  luna1.begin(115200);
  luna3.begin(115200);

  Serial.begin(115200);
  myTransfer.begin(Serial);
  ms5837.init();
  lsm6ds3.begin();
  qmc5883.init();
  lsm6ds3.calcGyroOffsets(3000);
  filter.begin(SAMPLE_RATE);
  escControl.calib(3000);
  Setpoint = 0;
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

  IMUSensorValue(CONSOLE_OFF, true);
  lunaDist1();
  lunaDist3();
  tflI2C.getData(Dist2, TFL_DEF_ADR);
  pres_sensor_values(CONSOLE_OFF);
  // Serial.print("Dist1: ");
  //Serial.println(Dist2);

  Comm();
  MotorDrive(CONSOLE_OFF, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD, FORWARD);
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
      Serial.print(gz);
      Serial.print(" magX: ");
      Serial.print(mx);
      Serial.print(" magY: ");
      Serial.print(my);
      Serial.print(" magZ: ");
      Serial.println(mz);

      consoleMilisPrevious = consoleMilisPrevious + consolePerReading;
    }
  }
}

void MotorDrive(bool console, bool RiseM1Direct, bool RiseM2Direct, bool FrontRightDirect, bool FrontLeftDirect, bool BackRightDirect, bool BackLeftDirect) {
  int RiseM1, RiseM2, FrontRight, FrontLeft, BackRight, BackLeft;

  RiseM1 = valueJoyStick_X_1;
  RiseM2 = valueJoyStick_X_1;
  FrontRight = 1500 + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);
  FrontLeft = 1500 + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  BackRight = 1500 + (valueJoyStick_X_2 - 1500) + (valueJoyStick_Y_2 - 1500) - (valueJoyStick_Y_1 - 1500);
  BackLeft = 1500 + (valueJoyStick_X_2 - 1500) - (valueJoyStick_Y_2 - 1500) + (valueJoyStick_Y_1 - 1500);

  escControl.escspeed(RiseM1, RiseM2, FrontRight, FrontLeft, BackRight, BackLeft);

  if (RiseM1Direct) RiseM1 -= 3000;
  if (RiseM2Direct) RiseM2 -= 3000;
  if (FrontRightDirect) FrontRight -= 3000;
  if (FrontLeftDirect) FrontLeft -= 3000;
  if (BackRightDirect) BackRight -= 3000;
  if (BackLeftDirect) BackLeft -= 3000;

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

void lunaDist1() {

  int uart[9];
  int check;

  if (luna1.available()) {       //check if serial port has data input
    if (luna1.read() == 0x59) {  //assess data package frame header 0x59
      uart[0] = 0x59;
      if (luna1.read() == 0x59) {  //assess data package frame header 0x59
        uart[1] = 0x59;
        for (int i = 2; i < 9; i++) {  //save data in array
          uart[i] = luna1.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {    //verify the received data as per protocol
          Dist1 = uart[2] + uart[3] * 256;  //calculate distance value
        }
      }
    }
  }
}

void lunaDist3() {

  int uart[9];
  int check;

  if (luna3.available()) {       //check if serial port has data input
    if (luna3.read() == 0x59) {  //assess data package frame header 0x59
      uart[0] = 0x59;
      if (luna3.read() == 0x59) {  //assess data package frame header 0x59
        uart[1] = 0x59;
        for (int i = 2; i < 9; i++) {  //save data in array
          uart[i] = luna3.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) {    //verify the received data as per protocol
          Dist3 = uart[2] + uart[3] * 256;  //calculate distance value
        }
      }
    }
  }
}

void Comm() {
  struct STRUCT {
    int32_t RollS = 0;
    int32_t PitchS = 0;
    int32_t HeadingS = 0;
    int32_t frontLidarS = 0;
    int32_t leftLidarS = 0;
    int32_t rightLidarS = 0;
    int32_t altitudeS = 0;
    int32_t data1S = 0;
    int32_t data2S = 0;
    int32_t data3S = 0;
    int32_t data4S = 0;
    int32_t data5S = 0;
  } rovDataTx;

  struct STRUCT1 {
    int32_t leftThumbX;
    int32_t leftThumbY;
    int32_t rightThumbX;
    int32_t rightThumbY;
    int32_t degree;
  } rovDataRx;

  rovDataTx.RollS = int(roll);
  rovDataTx.PitchS = int(pitch);
  rovDataTx.HeadingS = int(heading);
  rovDataTx.frontLidarS = Dist3;
  rovDataTx.leftLidarS = Dist2;
  rovDataTx.rightLidarS = Dist1;
  rovDataTx.altitudeS = int(altitude);
  rovDataTx.data1S = valueJoyStick_X_1;
  rovDataTx.data2S = 0;
  rovDataTx.data3S = 0;
  rovDataTx.data4S = 0;
  rovDataTx.data5S = 0;

  if (myTransfer.available()) {
    commMillisNow = millis();
    if (commMillisNow - commMilisPrevious >= commPerReading) {
      uint16_t recSize = 0;
      uint16_t sendSize = 0;

      recSize = myTransfer.rxObj(rovDataRx, recSize);

      sendSize = myTransfer.txObj(rovDataTx, sendSize);

      myTransfer.sendData(sendSize);
      commMilisPrevious = commMilisPrevious + commPerReading;
      //delay(50);
    }
    valueJoyStick_X_1 = rovDataRx.leftThumbX;
    valueJoyStick_Y_1 = rovDataRx.leftThumbY;
    valueJoyStick_X_2 = rovDataRx.rightThumbX;
    valueJoyStick_Y_2 = rovDataRx.rightThumbY;
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