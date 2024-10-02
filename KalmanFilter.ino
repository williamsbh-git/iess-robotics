#include <Wire.h>
#include <MPU6050.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

MPU6050 mpu;

// Define constants
const float T = 1.0;
const float pw = 5.0;
const float pb = 5.0;
const float qw = 100.0;
const float qb = 100.0;
const float rtheta = 5e-5;
const float rw = 0.3;

// Define matrices using the BasicLinearAlgebra library
BLA::Matrix<3, 3> Ad = {1, 0, 0, T, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> Qd = {T * qw, 0.5 * T * T * qw, 0, 0.5 * T * T * qw, (1.0 / 3) * T * T * T * qw, 0, 0, 0, T * qb};
BLA::Matrix<2, 2> Rd = {rtheta / T, 0, 0, rw / T};
BLA::Matrix<2, 3> Cd = {0, 1, 0, 1, 0, 1};
BLA::Matrix<3, 3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

BLA::Matrix<3> u = {0, 0, 0};
BLA::Matrix<2> y = {0, 0};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float yww = gx / 131.0; // Assuming 131 LSB/deg/s for 500 deg/s range
  float ywb = gy / 131.0;
  float ywn = gz / 131.0;

  float yar = ax / 4096.0; // Assuming 4096 LSB/g for 8g range
  float yat = ay / 4096.0;
  float yan = az / 4096.0;

  static BLA::Matrix<3> uPrev = {0, 0, 0};
  static BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

  float yt = atan2(-yar, yat) + yan;
  y(0) = yww;
  y(1) = yt;

  BLA::Matrix<3> uPredict = Ad * uPrev;

  BLA::Matrix<3, 3> Pnew = Ad * P * (~Ad) + Qd;

  BLA::Matrix<3, 2> CdT = ~Cd; // Transpose Cd to match dimensions

  BLA::Matrix<2, 2> S = Cd * Pnew * CdT + Rd;

  BLA::Matrix<3, 2> K = Pnew * CdT * Inverse(S);

  BLA::Matrix<2> yPredict = Cd * uPredict;

  BLA::Matrix<2> yDiff = y - yPredict;

  u = uPredict + K * yDiff;

  P = (I - K * Cd) * Pnew;

  uPrev = u;

  // Output the results
  Serial.print("Angular Velocity:");
  Serial.print(u(1));
  Serial.print(",Tilt Angle:");
  Serial.println(u(0));

  delay(100);
}