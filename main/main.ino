#include "vector.h"
#include <Servo.h>
#include <Arduino.h>
#include <TinyMPU6050.h>

#define MPU_PIN 2
#define LOG_LENGTH 128

#define gravity -9.81
#define targetHeight 250
#define velocityDelayFactor -1

#define neutralAngleA 90
#define neutralAngleB 90
#define neutralAngleC 90
#define neutralAngleD 90

#define angleMultiplierA 0.01
#define angleMultiplierB 0.01
#define angleMultiplierC 0.01
#define angleMultiplierD 0.01

#define maxAngle 20

Servo a;
Servo b;
Servo c;
Servo d;

MPU6050 mpu (Wire);

void setup() {
  // put your setup code here, to run once:
  a.attach(2);
  b.attach(3);
  c.attach(4);
  d.attach(5);

  mpu.Initialize();
  mpu.Calibrate();
}
struct Vector position = Vector(0, 0, 0);
struct Vector velocity = Vector(0, 0, 0);

unsigned long time = 0;
unsigned long lastLogTime = 0;

unsigned int logIndex;

bool isLogging = false;
bool isLaunched = false;
bool isLanded = false;

int positionX[LOG_LENGTH];
int positionY[LOG_LENGTH];
int positionZ[LOG_LENGTH];

int rotationX[LOG_LENGTH];
int rotationY[LOG_LENGTH];
int rotationZ[LOG_LENGTH];

void loop() {
  unsigned long newTime = micros();
  unsigned long deltaTime = newTime - time;
  if (newTime < time) {
    unsigned long minusTime = ~(unsigned long)0 - time;
    deltaTime = minusTime + newTime;
  }
  float dt = deltaTime * 0.000001f;
  struct Vector acceleration = getAcceleration();
  velocity = add(velocity, mul(acceleration, dt));
  position = add(position, mul(velocity, dt));

  if (isLaunched == false && acceleration.x > 30)
  {
    // the rocket has launched
    isLaunched = true;
    position = Vector(0, 0, 0);
    beginLogging();
  } else if (isLaunched == true && isLanded == false && velocity.x > -1 && velocity.x < 1) {
    // the rocket has landed
    isLanded = true;
    endLogging();
  }
  
  struct Vector angularV = getAngularVel();
  struct Vector angularP = getAngularPos();

  // d = 1/2 * a * t^2 + v * t + d0
  // newHeight = 1/2 * a * dt^2 + v * dt + p
  float tMax = -velocity.x / acceleration.x;
  float estimatedHeight = acceleration.x * 0.5 * tMax * tMax + velocity.x * tMax + position.x;
  float deltaHeight = targetHeight - estimatedHeight;

  setServoAngles(deltaHeight);

  if (isLogging) {
    if (newTime > lastLogTime) {
      logPositionAngle(position, angularP);
    }
  }
  
  time = newTime;
}

// Sets the servo angles based on a height delta.
void setServoAngles(float delta) {
  // Adjacent fins turn in opposite values relative to the servo.
  a.write(neutralAngleA + max(min(floor(angleMultiplierA * delta), maxAngle), 0));
  b.write(neutralAngleB - max(min(floor(angleMultiplierB * delta), maxAngle), 0));
  c.write(neutralAngleC + max(min(floor(angleMultiplierC * delta), maxAngle), 0));
  d.write(neutralAngleD - max(min(floor(angleMultiplierD * delta), maxAngle), 0));
}

void beginLogging() {
  isLogging = true;
  lastLogTime = time;
}
void endLogging() {
  isLogging = false;
}

void logPositionAngle(Vector position, Vector angle) {
  int pX = position.x * 100;
  int pY = position.y * 100;
  int pZ = position.z * 100;
  int aX = angle.x * 100;
  int aY = angle.y * 100;
  int aZ = angle.z * 100;

  positionX[logIndex] = pX;
  positionY[logIndex] = pY;
  positionZ[logIndex] = pZ;
  rotationX[logIndex] = aX;
  rotationY[logIndex] = aY;
  rotationZ[logIndex] = aZ;

  lastLogTime += 1e6;
  logIndex++;
  if (logIndex > LOG_LENGTH) {
    logIndex = 0;
  }
}

void setDrag(float ratio) {
  if (ratio > 1) {
    ratio = 1;
  } else if (ratio < 0) {
    // Uh oh! the rocket is trying to speed up!
    ratio = 0;
  }
}

struct Vector getAcceleration() {
  return Vector(mpu.GetAccX(), mpu.GetAccY(), mpu.GetAccZ());
}

struct Vector getAngularVel() {
  return Vector(mpu.GetGyroX(), mpu.GetGyroY(), mpu.GetGyroZ());
}

struct Vector getAngularPos() {
  return Vector(mpu.GetAngX(), mpu.GetAngY(), mpu.GetAngZ()); 
}