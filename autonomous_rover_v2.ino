#include "Wire.h"
#include <Servo.h> 
#include "I2Cdev.h"
#include <NewPing.h>
#include "MPU6050_6Axis_MotionApps20.h"

const int leftMotor []  = {3, 4};
const int rightMotor [] = {5, 6};

const int rearTrigPin  = A0;
const int rearEchoPin  = A1;
const int rightTrigPin = 7;
const int rightEchoPin = 8;
const int leftTrigPin  = 9;
const int leftEchoPin  = 10;
const int MAX_DISTANCE = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing rearSonar(rearTrigPin, rearEchoPin, MAX_DISTANCE); 
NewPing rightSonar(rightTrigPin, rightEchoPin, MAX_DISTANCE);
NewPing leftSonar(leftTrigPin, leftEchoPin, MAX_DISTANCE);

const int rearServoPin  = 11;
const int rightServoPin = 12;
const int leftServoPin  = 13;

const int SAFE_DISTANCE = 100;
const int STOP_DISTANCE = 25;

const int angleIndexTable[] = {0, 45, 90, 135, 180, -135, -90, -45};

long * frontDistances = new long [3];
long * rearDistances = new long [5];
long * distances180 = new long [5];
long * distances360 = new long [8];

int rightPos, leftPos, rearPos;
Servo rightServo;
Servo leftServo;
Servo rearServo;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float * mpuData;
float currentAngle;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void mpuSetup() {
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(42);
  mpu.setZGyroOffset(24);
  mpu.setXAccelOffset(-1316);
  mpu.setYAccelOffset(577);
  mpu.setZAccelOffset(1335);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

float * getMPUData() {
  // clear extra packets between calls
  mpu.resetFIFO();
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {}
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  // convert from rads to degrees
  for (int i = 0; i < 3; i++) {
    ypr[i] *= 180/M_PI;
  }
  return ypr;
}

void calibrateRover() {
  distances360 = get360Distances();

  for (int i = 0; i < 8; i++) {
    Serial.print("Angle: ");
    Serial.print(i * 45);
    Serial.print("    Distance: ");
    Serial.println(distances360[i]);
  }

  int angle = angleIndexTable[maxIndex(distances360, 8)];

  mpuData = getMPUData();
  currentAngle = mpuData[0];
  Serial.print("Aiming towards: ");
  Serial.println(angle);

  if (angle == 0)
    return;
  else if (angle > 0 && angle < 180) {
    right();
    while (currentAngle < angle - 5) {
      mpuData = getMPUData();
      currentAngle = mpuData[0];
      Serial.println(currentAngle);
    }
  }
  else {
    left();
    while (currentAngle > angle + 5) {
        mpuData = getMPUData();
        currentAngle = mpuData[0];
        Serial.println(currentAngle);
    }
  }
  
  stop();
}

long * get360Distances() {
  long * distances = new long [8];

  for (rightPos = 0, leftPos = 180, rearPos = 45; rightPos <= 90, leftPos >= 90, rearPos <= 135; rightPos += 45, leftPos -= 45, rearPos += 45) {
    leftServo.write(leftPos);
    rightServo.write(rightPos);
    rearServo.write(rearPos);
    if (leftPos == rightPos) {
        distances[2] = (getDistance(leftTrigPin, leftEchoPin + getDistance(rightTrigPin, rightEchoPin))) / 2;
    } else {
      distances[leftPos  / 45] = getDistance(leftTrigPin, leftEchoPin);
      distances[rightPos / 45] = getDistance(rightTrigPin, rightEchoPin);
    } 
    distances[(rearPos + 180) / 45] = getDistance(rearTrigPin, rearEchoPin);
    delay(250);
  }
  rearServo.write(90);
  
  return distances;
}

long * getFrontDistances() {  
  long * distances = new long [3];
  for (rightPos = 45, leftPos = 135; rightPos <= 90, leftPos >= 90 ; rightPos += 45, leftPos -= 45) {
    leftServo.write(leftPos);
    rightServo.write(rightPos);
    if (leftPos == rightPos) {
      distances[1] = (getDistance(leftTrigPin, leftEchoPin + getDistance(rightTrigPin, rightEchoPin))) / 2;
    } else {
      distances[(leftPos  / 45) - 1] = getDistance(leftTrigPin, leftEchoPin);
      distances[(rightPos / 45) - 1] = getDistance(rightTrigPin, rightEchoPin);
    } 
    delay(250);
  }
  return distances;
}

long * getRearDistances() {  
  long * distances = new long [5];
  for (rearPos = 0; rearPos <= 180; rearPos += 45) {
    rearServo.write(rearPos);
    distances[rearPos  / 45] = getDistance(rearTrigPin, rearEchoPin);
    delay(250);
  }
  return distances;
}

void driveAndAvoid() {
  // crappy stop, turn, and go motion. TODO: add turn while driving to evade
  frontDistances = getFrontDistances();
  mpuData = getMPUData();
  currentAngle = mpuData[0];
  int startingAngle = currentAngle;
  if (frontDistances[1] >= SAFE_DISTANCE) {
    forward();
  } else if (frontDistances[0] >= SAFE_DISTANCE || frontDistances[2] >= SAFE_DISTANCE) {
    stop();
    if (frontDistances[0] > frontDistances[2])
      left();
    else
      right();
    while (abs(startingAngle - currentAngle) < 40) {
      mpuData = getMPUData();
      currentAngle = mpuData[0];
    }
  } else {
    stop();
    rearDistances = getRearDistances();
    if (rearDistances[2] >= SAFE_DISTANCE) {
      backward();
      while (frontDistances[1] < SAFE_DISTANCE) {
       frontDistances = getFrontDistances();  
      }
      stop();
    }
    distances360 = get360Distances();
    int angle = maxIndex(distances360, 8) * 45;
    right();
    while (startingAngle + abs(startingAngle - currentAngle) < angle) {
      mpuData = getMPUData();
      currentAngle = mpuData[0];
    }
    stop();
  }
}

// returns distance in cm
long getDistance(int trigPin, int echoPin) {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  long duration = pulseIn(echoPin, HIGH);
  // convert duration to cm 
  return (duration / 2) / 29.1; // travels distance twice, divide by 2
}

int maxIndex(long* arr, int size) {
 int maxIndex = 0;
 long max = arr[0];
 for (int i = 1; i < size; i++) {
   if (max < arr[i]){
     max = arr[i];
     maxIndex = i;
   }
 }
 return maxIndex;
}

// ------ Motor Control -------- //
void right() {
  digitalWrite(leftMotor[0], HIGH); 
  digitalWrite(leftMotor[1], LOW); 
  digitalWrite(rightMotor[0], HIGH); 
  digitalWrite(rightMotor[1], LOW); 
}

void left(){
  digitalWrite(leftMotor[0], LOW); 
  digitalWrite(leftMotor[1], HIGH); 
  digitalWrite(rightMotor[0], LOW); 
  digitalWrite(rightMotor[1], HIGH); 
}

void forward(){
  digitalWrite(leftMotor[0], LOW); 
  digitalWrite(leftMotor[1], HIGH); 
  digitalWrite(rightMotor[0], HIGH); 
  digitalWrite(rightMotor[1], LOW);
}

void backward(){
  digitalWrite(leftMotor[0], HIGH); 
  digitalWrite(leftMotor[1], LOW); 
  digitalWrite(rightMotor[0], LOW); 
  digitalWrite(rightMotor[1], HIGH); 
}

void stop() {
  digitalWrite(leftMotor[0], LOW); 
  digitalWrite(leftMotor[1], LOW); 
  digitalWrite(rightMotor[0], LOW); 
  digitalWrite(rightMotor[1], LOW); 
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpuSetup();
  rearServo.attach(rearServoPin);
  rightServo.attach(rightServoPin);
  leftServo.attach(leftServoPin);
  pinMode(leftEchoPin, INPUT);
  pinMode(leftTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rearEchoPin, INPUT);
  pinMode(rearTrigPin, OUTPUT);
  for (int i = 0; i < 2; i++){
    pinMode(leftMotor[i], OUTPUT);
    pinMode(rightMotor[i], OUTPUT);
  }

  calibrateRover();
}

void loop() {
  // if programming of mpu failed, don't try to do anything
  if (!dmpReady) {
    stop();
    return;
  }
  // driveAndAvoid(); 
}
