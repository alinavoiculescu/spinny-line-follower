#include <QTRSensors.h>

const int bitsPerSecond = 9600;
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
const float maxSpeed = 255;
const float minSpeed = -255;
const float baseSpeed = 255;
const int sensorCount = 6;
const int delayBeforeCalibration = 500;
const int calibrationCount = 350;
const int sensorMinValue = 0;
const int sensorMaxValue = 5000;
const int errorMinValue = -70;
const int errorMaxValue = 70;
const int calibrationSpeedLower = -255;
const int calibrationSpeedZero = 0;
const int calibrationSpeedHigher = 255;
const int errorIntervalLowerBound = -30;
const int errorIntervalUpperBound = 30;
const byte right = 0;
const byte left = 1;

float kp = 20;
float ki = 0.1;
float kd = 17;
float m1Speed = 0;
float m2Speed = 0;
float p = 0;
float i = 0;
float d = 0;
float error = 0;
float lastError = 0;
int sensorValues[sensorCount];
int sensors[sensorCount] = {0, 0, 0, 0, 0, 0};
float motorSpeed;
byte lastDirection = 0;
QTRSensors qtr;

void setup() {
  Serial.begin(bitsPerSecond);
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  delay(delayBeforeCalibration);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  for (uint16_t i = 0; i < calibrationCount; i++) {
    qtr.calibrate();
    Serial.println(getError());
    if (getError() > errorIntervalUpperBound) {
      lastDirection = right;
      setMotorSpeed(calibrationSpeedZero, calibrationSpeedLower);
    }
    else if (getError() < errorIntervalLowerBound) {
      lastDirection = left;
      setMotorSpeed(calibrationSpeedLower, calibrationSpeedZero);
    }
    else {
      if (lastDirection == right) {
        setMotorSpeed(calibrationSpeedHigher, calibrationSpeedZero);
      }
      else if (lastDirection == left) {
        setMotorSpeed(calibrationSpeedZero, calibrationSpeedHigher);
      }
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  float error = getError();

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;

  motorSpeed = kp * p + ki * i + kd * d;
  
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  }
  else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

  setMotorSpeed(m1Speed, m2Speed);
}

float getError() {
  return map(qtr.readLineBlack(sensorValues), sensorMinValue, sensorMaxValue, errorMinValue, errorMaxValue);
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
  motor1Speed = -motor1Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  }
  else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  }
  else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}