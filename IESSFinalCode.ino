#include <Wire.h>
#include <MPU6050.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

MPU6050 mpu;

#define PI 3.1415926535897932384626433832795

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

unsigned long previousMillis = 0; 

const float T = 0.01;
const float pw = 200.0;
const float pb = 300.0;
const float qw = 3200.0;
const float qb = 3000.0;
const float rtheta = 5.5;
const float rw = 3.85;

const float Ra = 14.69;    //armature resistance (Ohms)
const float La = 2.5e-3;  //armature inductance (H)
const float Km = .191;   //motor speed constant (V/rad/s)
const float eta = .50;   //gearbox efficiency (%)
const float tm = 0.0095; 

float previousValue = 0.0;
float E;
float Current;
float phi = 0;
float dphi = 0;

BLA::Matrix<3> uPrev = {0, 0, 0};
BLA::Matrix<3, 3> P = {T * pw, 0.5 * pw * T * T, 0, 0, (1.0 / 3) * T * T * T * pw, 0, 0, 0, T * pb};

// Define matrices using the BasicLinearAlgebra library
BLA::Matrix<3, 3> Ad = {1, 0, 0, T, 1, 0, 0, 0, 1};
BLA::Matrix<3, 3> Qd = {T * qw, 0.5 * T * T * qw, 0, 0.5 * T * T * qw, (1.0 / 3) * T * T * T * qw, 0, 0, 0, T * qb};
BLA::Matrix<2, 2> Rd = {rtheta / T, 0, 0, rw / T};
BLA::Matrix<2, 3> Cd = {0, 1, 0, 1, 0, 1};
BLA::Matrix<3, 3> I = {1, 0, 0, 0, 1, 0, 0, 0, 1};

BLA::Matrix<3> u = {0, 0, 0};
BLA::Matrix<2> y = {0, 0};
BLA::Matrix<1, 4> Klqr = {-0.1, -250, -.02, -60};

BLA::Matrix<3, 1> KalmanFilter(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz);
float requiredVoltage(float deltaphi, float tau);
int voltageToDutyCycle(float voltage, float maxVoltage);
bool changedDirection(float currentValue);
float smoothControl(float currentVoltage, float previousVoltage);


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
  Serial.begin(57600);

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

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_isrAB, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), encoder_isrCD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), encoder_isrCD, CHANGE);

  Wire.begin();
  mpu.initialize();
}
int interval = 50;
void loop() {
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    unsigned long current_time = micros();
    float dt_interrupt = (float)(current_time - last_time) / 1000000;

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

    phi = (angleAB + angleCD)/2;
    dphi = (angular_velocityAB+angular_velocityCD)/2;

    // Debug: Print encoder values
    // Kalman filter
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    BLA::Matrix<3, 1> predictedValues = KalmanFilter(ax,ay,az,gx,gy,gz);

    float predictedTheta = predictedValues(1);
    float predictedOmega = predictedValues(0);

    BLA::Matrix<4, 1> states = {phi, predictedTheta, dphi, predictedOmega};
    float predictTheta = states(1);

    BLA::Matrix<4,1> refer = {0,0.183,0,0};
    BLA::Matrix<4,1> error = refer - states;

    BLA::Matrix<1> Kx = Klqr * error;

    float tau = Kx(0);
    float omega = states(3);
    // Debug: Print tau val

    float V = requiredVoltage(tau, dphi);

    // Debug: Print voltage value
    //Serial.println(predictTheta);
    //differentValue(V, predictTheta);
    //Serial.println(states(2));
    bool turned = changedDirection(predictTheta);

    int speed = voltageToDutyCycle(abs(V),12);
    int threshold = 0.2;

    // if (omega > 30 || omega < -30) {
    //   speed += 20;
    // }

    //bool changedTheta = changedDirection(predictTheta);

    // if (changedTheta == true) {
    //   predictTheta = previousValue;
    // }

    // if (predictTheta < 0.04 && predictTheta > -0.04) {
    //   predictTheta = 0;
    // }
    
    float thetaError = error(1);
    speed *= abs(thetaError)/1.5;
   

    if (thetaError < 0.01 && thetaError > -0.005) {
      thetaError = 0;
    }

    speed += 70;

    if (thetaError < 0) {
      analogWrite(M1A, 0);
      analogWrite(M1B, 30+speed);
      analogWrite(M2A, 0);
      analogWrite(M2B, 30+speed);
    } 
    else if (thetaError > 0) {
      analogWrite(M1A, speed);
      analogWrite(M1B, 0);
      analogWrite(M2A, speed);
      analogWrite(M2B, 0);
    } 
    // else if (V < 0 && predictTheta == 0) {
    //   analogWrite(M1A, 0);
    //   analogWrite(M1B, 20);
    //   analogWrite(M2A, 0);
    //   analogWrite(M2B, 20);
    // }
    // else if (V > 0 && predictTheta == 0) {
    //   analogWrite(M1A, 20);
    //   analogWrite(M1B, 0);
    //   analogWrite(M2A, 20);
    //   analogWrite(M2B, 0);
    // }
    else if (thetaError == 0) {
      analogWrite(M1A, 0);
      analogWrite(M1B, 0);
      analogWrite(M2A, 0);
      analogWrite(M2B, 0);
    }
    
    last_time = current_time;
    Serial.println(predictTheta);
  }
}

BLA::Matrix<3, 1> KalmanFilter(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz) {
  float yww = gx / 131.0; // Assuming 131 LSB/deg/s for 500 deg/s range
  float ywb = gy / 131.0;
  float ywn = gz / 131.0;

  float yan = az / 16384.0;
  float yar = ax / 16384.0;
  float yat = ay / 16384.0;

  float yt = atan2(yan, -yat);
  y(1) = yww;
  y(0) = yt;
  //Serial.println(yt);
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
    Current = (1 / Km) * ((tau / 2*((eta * GEAR_RATIO)) - tm));
  } else if (tau < 0) {
    Current = (1 / Km) * ((tau / 2*((eta * GEAR_RATIO)) + tm));
  } else if (tau == 0) {
    Current = 0;
  }
  E = (Current * Ra + Km * deltaphi);

  if (E > 12) {
    E = 12;
  }
  else if (E < -12) {
    E = -12;
  }
  else if (E = Km * deltaphi) {
    E = 0;
  }
  return E;
}

bool changedDirection(float currentValue) {
  bool changed = (currentValue * previousValue < 0); // True if signs are different
  previousValue = currentValue; // Update previous voltage
  return changed;
}

void differentValue(float one, float two) {
  if (one * two < 0 && one < 0) {
    Serial.println("one -");
  }
  else if (one * two < 0 && two < 0) {
    Serial.println("two -");
  }
  else if (one * two > 0 && one > 0) {
    Serial.println("both +");
  }
  else if (one * two > 0 && one < 0) {
    Serial.println("both -");
  }
}

float smoothControl(float currentVoltage, float previousVoltage) {
  const float alpha = 0.2; // Smoothing factor, adjust between 0 and 1
  return alpha * currentVoltage + (1 - alpha) * previousVoltage;
}

int voltageToDutyCycle(float voltage, float maxVoltage) {
  // Ensure the voltage is within the expected range
  if (voltage > maxVoltage) voltage = maxVoltage;
  if (voltage < -maxVoltage) voltage = -maxVoltage;

  // Map the voltage to the range [0, 255]
  int dutyCycle = int((voltage / maxVoltage) * 255.0);
  
  // Ensure the duty cycle is within [0, 255]
  if (dutyCycle > 255) dutyCycle = 255;
  if (dutyCycle < 0) dutyCycle = 0;

  return dutyCycle;
}

void ensureOppositeSigns(float& a, float& b) {
    // If both values have the same sign, invert the sign of b
    if ((a >= 0 && b >= 0) || (a < 0 && b < 0)) {
        b = -b;
    }
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

// What's working:

// motors go in right directions
// meausurements seem to be accurate
// Kalman seems to be working correctly
// robot is correcting itself
// dead zone around zero point is effective

// What's not working:

// motors are overcorrecting
// jerking a lot
// zero point is impossible to reach
// always goes too far when balancing and thus never balances

// What could fix it:

// K, R, Q and P values
// duty cycle values
// logic

