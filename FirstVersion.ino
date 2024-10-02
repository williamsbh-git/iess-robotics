#include <Wire.h>
#include <MPU6050.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

MPU6050 mpu;

#define M1A 7
#define M1B 6
#define M2A 10 
#define M2B 9

#define ENCODER_A 2
#define ENCODER_B 3
#define ENCODER_C 11
#define ENCODER_D 12

volatile uint8_t encoderALast;
volatile uint8_t encoderBLast;
volatile uint8_t encoderCLast;
volatile uint8_t encoderDLast;

#undef F_CPU
#define F_CPU 16000000UL // 16MHz clock frequency
#define PRESCALER 8UL
#define F_S 100UL  // 100Hz sampling frequency

volatile uint8_t countFlag = 0;
int counts = F_CPU / (PRESCALER * F_S) - 1;
volatile bool Mode = 0;

const float WHEEL_RADIUS = 0.021;  // Radius of the wheel in meters
const float GEAR_RATIO = 50.0;  // Gear ratio of the encoder

const int ENCODER_PULSE_PER_REVOLUTION = 28;  // Number of encoder pulses per revolution
const float COUNTS_PER_REVOLUTION = ENCODER_PULSE_PER_REVOLUTION * GEAR_RATIO; // Pulses per revolution

// Define the variables for the encoder
volatile int16_t encoder_countsAB = 0;
volatile int16_t encoder_countsCD = 0;

volatile unsigned long last_time = 0;
volatile float position_changeAB = 0.0;
volatile float position_changeCD = 0.0;

volatile float angular_velocityAB = 0.0;
volatile float angular_velocityCD = 0.0;

volatile float angleAB = 0.0;
volatile float angleCD = 0.0;

unsigned long previousMillis = 0; // Store the last time a sample was taken
const long interval = 500;

const float T = 1.0;
const float pw = 200.0;
const float pb = 200.0;
const float qw = 200.0;
const float qb = 200.0;
const float rtheta = .5;
const float rw = 3;

const float Ra = 14.69;    //armature resistance (Ohms)
const float La = 2.5e-3;  //armature inductance (H)
const float Km = .191;   //motor speed constant (V/rad/s)
const float eta = .60;   //gearbox efficiency (%)
const float tm = 0.0095; 

float E;
float Current;

// Define matrices using the BasicLinearAlgebra library
BLA::Matrix<3, 3> Ad = {1, 0, 0, T, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> Qd = {T * qw, 0.5 * T * T * qw, 0, 0.5 * T * T * qw, (1.0 / 3) * T * T * T * qw, 0, 0, 0, T * qb};
BLA::Matrix<2, 2> Rd = {rtheta / T, 0, 0, rw / T};
BLA::Matrix<2, 3> Cd = {0, 1, 0, 1, 0, 1};
BLA::Matrix<3, 3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

BLA::Matrix<3> u = {0, 0, 0};
BLA::Matrix<2> y = {0, 0};
BLA::Matrix<1, 4> Klqr = {-0.001, -250, -0.5, -25};

BLA::Matrix<3, 1> KalmanFilter(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
float requiredVoltage(float deltaphi, float tau);

void setup() {
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  TCCR5A = 0;
  TCCR5B = (1 << WGM52) | (0 << CS52) | (1 << CS51) | (0 << CS50);
  OCR5A = counts;
  TIMSK5 |= (1 << OCIE5A);
  TCNT5 = 0;

  PCMSK2 |= (1 << PCINT18); // For ENCODER_A
  PCMSK2 |= (1 << PCINT19); // For ENCODER_B
  PCMSK1 |= (1 << PCINT10); // For ENCODER_C
  PCMSK1 |= (1 << PCINT11); // For ENCODER_D
  PCICR |= (1 << PCIE2);
  PCICR |= (1 << PCIE1);

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isrAB, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoder_isrCD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), encoder_isrCD, CHANGE);

  Wire.begin();
  mpu.initialize();
}

void loop() {
  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  unsigned long current_time = micros();
  // Calculate the time difference since the last interrupt
  float dt_interrupt = (float)(current_time - last_time) / 1000000;

  // Calculate the change in the position of the encoder in radians
  position_changeAB = 2 * PI * encoder_countsAB / COUNTS_PER_REVOLUTION;
  position_changeCD = 2 * PI * encoder_countsCD / COUNTS_PER_REVOLUTION;

  // Reset encoder counts
  encoder_countsAB = 0;
  encoder_countsCD = 0;

  // Calculate the angular velocity
  angular_velocityAB = position_changeAB / dt_interrupt;
  angular_velocityCD = position_changeCD / dt_interrupt;

  // Calculate the angle
  angleAB += position_changeAB;
  angleCD += position_changeCD;

  last_time = current_time;


  float phi = angleAB;
  float dphi = angular_velocityAB;

  // Debug: Print encoder values
  // Kalman filter
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  BLA::Matrix<3, 1> predictedValues = KalmanFilter(ax,ay,az,gx,gy,gz);

  float predictedTheta = predictedValues(0);
  float predictedOmega = predictedValues(1);

  BLA::Matrix<4, 1> states = {phi, dphi, predictedTheta, predictedOmega};
  BLA::Matrix<1> Kx = Klqr * states;

  float tau = Kx(0);

  // Debug: Print tau val

  float V = requiredVoltage(tau, dphi);

  // Debug: Print voltage value
  Serial.println(V);
  Serial.println(tau);

  int speed = 75;
  int slowerspeed = 50;
  int threshold = 40;

  // Use the calculated voltage to control the motors
  if (V > threshold) {
    analogWrite(M1A, 0);
    analogWrite(M1B, speed);
    analogWrite(M2A, 0);
    analogWrite(M2B, speed);
  } 
  else if (V < -threshold) {
    analogWrite(M1A, speed);
    analogWrite(M1B, 0);
    analogWrite(M2A, speed);
    analogWrite(M2B, 0);
  } 
  else if (V < threshold && V > 0) {
    analogWrite(M1A, 0);
    analogWrite(M1B, slowerspeed);
    analogWrite(M2A, 0);
    analogWrite(M2B, slowerspeed);
  }
  else if (V > -threshold && V < 0) {
    analogWrite(M1A, slowerspeed);
    analogWrite(M1B, 0);
    analogWrite(M2A, slowerspeed);
    analogWrite(M2B, 0);
  }
}


BLA::Matrix<3, 1> KalmanFilter(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  float yww = gx / 131.0; // Assuming 131 LSB/deg/s for 500 deg/s range
  float ywb = gy / 131.0;
  float ywn = gz / 131.0;

  // Assuming 4096 LSB/g for 8g range
  float yan = az / 4096.0;
  float yar = ax / 4096.0;
  float yat = ay / 4096.0;

  static BLA::Matrix<3> uPrev = {0, 0.15, 0};
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

  return u;
}

float requiredVoltage(float deltaphi, float tau) {
  if (tau > 0) {
    Current = (1 / Km) * ((tau / (eta * GEAR_RATIO)) - tm);
  } else if (tau < 0) {
    Current = (1 / Km) * ((tau / (eta * GEAR_RATIO)) + tm);
  } else if (tau == 0) {
    Current = 0;
  }
  E = Current * Ra + Km * deltaphi;
  return E;
}

void encoder_isrAB() {
  uint8_t encoderAState = digitalRead(ENCODER_A);
  uint8_t encoderBState = digitalRead(ENCODER_B);

  if (encoderALast*encoderBLast) { //State 11
    encoder_countsAB -= (encoderAState - encoderBState); 
  } else if (encoderALast && (encoderALast^encoderBLast)) { //State 10
    encoder_countsAB += encoderAState*encoderBState - (1-encoderAState|encoderBState); 
  } else if (encoderBLast && (encoderBLast^encoderALast)) { //State 01
    encoder_countsAB += (1-encoderAState|encoderBState) - encoderAState*encoderBState;
  } else { //State 00
    encoder_countsAB += encoderAState - encoderBState; 
  }

  encoderALast = encoderAState;
  encoderBLast = encoderBState;
}

void encoder_isrCD() {
  uint8_t encoderCState = digitalRead(ENCODER_C);
  uint8_t encoderDState = digitalRead(ENCODER_D);

  if (encoderCLast*encoderDLast) { //State 11
    encoder_countsCD -= (encoderCState - encoderDState); 
  } else if (encoderCLast && (encoderCLast^encoderDLast)) { //State 10
    encoder_countsCD += encoderCState*encoderDState - (1-encoderCState|encoderDState); 
  } else if (encoderDLast && (encoderDLast^encoderCLast)) { //State 01
    encoder_countsCD += (1-encoderCState|encoderDState) - encoderCState*encoderDState;
  } else { //State 00
    encoder_countsCD += encoderCState - encoderDState; 
  }

  encoderCLast = encoderCState;
  encoderDLast = encoderDState;
}

ISR(PCINT2_vect) {
  encoder_isrAB();
}

ISR(PCINT1_vect) {
  encoder_isrCD();
}

ISR(TIMER5_COMPA_vect) {  
  countFlag = 1;
  Mode = !Mode;
}
