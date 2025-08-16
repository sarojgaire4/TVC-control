#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

Servo servoX, servoY, roll1, roll2;

const int pinX = 13, pinY = 12, pinRoll1 = 14, pinRoll2 = 27;
const int ledPin = 2;

Adafruit_MPU6050 mpu;

const int center = 90;
const int radius = 58;
const int rollAmplitude = 38;
const float tiltRange = 15.0;
const float gimbalDeadzone = 3.0;
const float alpha = 0.98;

int lastX = -1, lastY = -1, lastRoll = -1;

float gyroZoffset = 0;
float compPitch = 0, compRoll = 0;

unsigned long lastUpdate = 0;
unsigned long startTime;
bool demoDone = false, centerPauseDone = false;

// PID constants (easily changeable)
double Kp = 4.0, Ki = 0.5, Kd = 0.05;

// PID variables
double pitchSetpoint = 0, pitchInput = 0, pitchOutput = 0;
double rollSetpoint = 0, rollInput = 0, rollOutput = 0;
double yawSetpoint = 0, yawInput = 0, yawOutput = 0;

PID pidPitch(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID pidRoll(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);
PID pidYaw(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep MPU6050 still.");
  float sum = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(2);
  }
  gyroZoffset = sum / 500;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZoffset * 57.2958);
}

void runDemoMotion(float elapsed) {
  int demoX = center + radius * sin(elapsed);
  int demoY = center + radius * cos(elapsed);
  int demoRoll = center + rollAmplitude * sin(elapsed * 2);

  servoX.write(demoX);
  servoY.write(demoY);
  roll1.write(demoRoll);
  roll2.write(demoRoll);

  digitalWrite(ledPin, (millis() / 500) % 2 == 0 ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  roll1.setPeriodHertz(50);
  roll2.setPeriodHertz(50);

  servoX.attach(pinX);
  servoY.attach(pinY);
  roll1.attach(pinRoll1);
  roll2.attach(pinRoll2);

  servoX.write(center);
  servoY.write(center);
  roll1.write(center);
  roll2.write(center);

  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);

  pidPitch.SetOutputLimits(-radius, radius);
  pidRoll.SetOutputLimits(-radius, radius);
  pidYaw.SetOutputLimits(-rollAmplitude, rollAmplitude);

  pidPitch.SetTunings(Kp, Ki, Kd);
  pidRoll.SetTunings(Kp, Ki, Kd);
  pidYaw.SetTunings(Kp, Ki, Kd);

  delay(2000);
  startTime = millis();
  lastUpdate = millis();
}

void loop() {
  unsigned long now = millis();
  float elapsed = (now - startTime) / 1000.0;

  if (!demoDone) {
    runDemoMotion(elapsed);
    if (elapsed > 10) {
      demoDone = true;
      digitalWrite(ledPin, LOW);
      servoX.write(center);
      servoY.write(center);
      roll1.write(center);
      roll2.write(center);
      startTime = millis();
      lastUpdate = millis();
    }
    delay(10);
    return;
  }

  if (!centerPauseDone) {
    if (elapsed > 2) {
      centerPauseDone = true;
      calibrateGyro();
    }
    return;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float dt = (millis() - lastUpdate) / 1000.0;
  lastUpdate = millis();

  float accelPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  float accelRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float gyroPitchRate = g.gyro.y * 57.2958;
  float gyroRollRate  = g.gyro.x * 57.2958;
  float gyroZrate     = (g.gyro.z - gyroZoffset) * 57.2958;
  if (abs(gyroZrate) < 0.8) gyroZrate = 0;

  compPitch = alpha * (compPitch + gyroPitchRate * dt) + (1 - alpha) * accelPitch;
  compRoll  = alpha * (compRoll  + gyroRollRate  * dt) + (1 - alpha) * accelRoll;

  compPitch = constrain(compPitch, -tiltRange, tiltRange);
  compRoll  = constrain(compRoll, -tiltRange, tiltRange);

  // PID inputs and compute
  pitchInput = compPitch;
  rollInput = compRoll;
  yawInput = gyroZrate;

  pidPitch.Compute();
  pidRoll.Compute();
  pidYaw.Compute();

  int servoPitch = center - (int)pitchOutput;
  int servoRoll  = center - (int)rollOutput;
  int servoYaw   = center - (int)yawOutput;

  if (servoPitch != lastX) {
    servoX.write(servoPitch);
    lastX = servoPitch;
  }
  if (servoRoll != lastY) {
    servoY.write(servoRoll);
    lastY = servoRoll;
  }
  if (servoYaw != lastRoll) {
    roll1.write(servoYaw);
    roll2.write(servoYaw);
    lastRoll = servoYaw;
  }

  Serial.print("Pitch Angle: "); Serial.print(compPitch);
  Serial.print(" | Roll Angle: "); Serial.print(compRoll);
  Serial.print(" | Yaw (Gyro Z): "); Serial.println(gyroZrate);

  delay(5);
}
