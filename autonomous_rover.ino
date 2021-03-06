#include <Servo.h> 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

Servo servo;
MPU6050 mpu;
const int servoPin = 10;
const int trigPin  = 11;
const int echoPin  = 12;
const int leftMotor []  = {4, 5};
const int rightMotor [] = {6, 7};

long * distances180 = new long [5];
long * distances360 = new long [8];

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
  
//  int angle = maxIndex(distances360, 8) * 45;
}

long * get360Distances() {
  int idx = 0;
  long * distances = new long [8];
  for (int i = 0; i <= 180; i += 45) {
    servo.write(i);
    distances[idx++] = getDistance();
    delay(250);
  }

  right();
  // TODO: Replace timed rotation angle using rpm by gyro/accel
  delay(450);
  stop();
  
  for (int i = 180; i >= 45; i -= 45) {
    servo.write(i);
    distances[idx++] = getDistance();
    delay(250);
  }

  servo.write(90);
  
  return distances;
}

long * get180Distances() {
  long * distances = new long [5]; // store distances in 45 degree increments
  for (int i = 0; i <= 5; i++) {
    servo.write(i * 45);
    distances[i] = getDistance();
    delay(250);
  }
  
  servo.write(90);
  
  return distances;
}

// returns distance in cm
long getDistance() {
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
  servo.attach(servoPin);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  for (int i = 0; i < 2; i++){
    pinMode(leftMotor[i], OUTPUT);
    pinMode(rightMotor[i], OUTPUT);
  }

  calibrateRover();
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
    stop();
    return;
  }
}

