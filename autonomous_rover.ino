#include <Servo.h> 

Servo servo;
const int servoPin = 10;
const int trigPin  = 11;
const int echoPin  = 12;
const int leftMotor []  = {4, 5};
const int rightMotor [] = {6, 7};

void sweep() { 
  int pos = 0;
  for(pos = 0; pos <= 180; pos += 10) {
    servo.write(pos);
    delay(100);
  }
  for(pos = 180; pos >= 0; pos -= 10) {
    servo.write(pos);
    delay(100);
  } 
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

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  for (int i = 0; i < 2; i++){
    pinMode(leftMotor[i], OUTPUT);
    pinMode(rightMotor[i], OUTPUT);
  }
}

void loop() {
  
}

