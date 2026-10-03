/*
  Self Balancing Robot - ESP32-S3 port (DMP-based)
  --------------------------------------------------
  Ported from the classic MPU6050 DMP + PID_v1 + LMotorController reference
  sketch. Same libraries, same overall structure. Changes made ONLY where
  required for ESP32-S3 compatibility:
    - TWBR (AVR-only register) replaced with Wire.setClock(400000)
    - attachInterrupt() updated to ESP32 GPIO-pin syntax
    - Added an INTERRUPT_PIN definition (wire MPU6050 INT to this GPIO)
  Wiring assumes L298N (ENA/IN1/IN2/ENB/IN3/IN4).

  BEFORE THIS WILL BALANCE, YOU MUST:
  1. Find your real `setpoint` value (see calibration note below) - 181.0 was
     tuned for the original build's MPU6050 mounting orientation, not yours.
  2. Retune Kp/Ki/Kd for your 200RPM BO motors + 65mm wheels - the values
     below are the reference sketch's originals, just a starting point.
  3. Re-measure MIN_ABS_SPEED (deadzone) for your motors - see earlier
     deadzone calibration method (ramp PWM up from 0 until wheel just turns).
  4. For best accuracy, run the standard MPU6050 "IMU_Zero" example sketch
     once to get real gyro/accel offsets, then replace the setXGyroOffset()
     etc. calls below with your actual measured values instead of 0.

  SETPOINT CALIBRATION:
    Temporarily comment out the pid.Compute()/motorController.move() lines
    in loop(), add Serial.println(input) instead, hold the robot perfectly
    upright by hand, and read what value input settles at. That's your real
    setpoint - replace originalSetpoint with it.
*/

#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30   // PLACEHOLDER - re-measure for your BO motors

// ---------- ESP32-S3 pin map (adjust to your board) ----------
#define SDA_PIN 8
#define SCL_PIN 9
#define INTERRUPT_PIN 2   // wire MPU6050's INT output to this GPIO

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 165.0; // PLACEHOLDER - calibrate for your mounting, see notes above
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

// starting gains from the reference sketch - RETUNE for your motors/wheels
double Kp = 85;
double Kd = 2.2;
double Ki = 260;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

// MOTOR CONTROLLER - L298N pin mapping (adjust to your wiring)
int ENA = 6;   // PWM enable, left motor
int IN1 = 1;
int IN2 = 3;
int IN3 = 4;
int IN4 = 5;
int ENB = 7;   // PWM enable, right motor
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // ESP32 equivalent of the original TWBR = 24 line
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  // PLACEHOLDER offsets - run the MPU6050 IMU_Zero example sketch once to
  // get real values for your specific sensor, then put them here.
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    // ESP32 interrupt syntax (was: attachInterrupt(0, dmpDataReady, RISING))
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
  }
}
