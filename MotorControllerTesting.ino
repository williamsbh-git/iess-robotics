#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize(); //Initializes MPU for measurements +/-250dps and +/-2g
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("");
  Serial.print(ax/16384.0); Serial.print(" "); //Scale with 2^-14 to convert readings into +/- 2g
  Serial.print(ay/16384.0); Serial.print(" ");
  Serial.println(az/16384.0);
  Serial.print("");
  Serial.print(gx/131.0); Serial.print(" "); //Scale with 250/2^15 to convert readings into +/- 250dps
  Serial.print(gy/131.0); Serial.print(" ");
  Serial.println(gz/131.0);
  delay(1000);
}