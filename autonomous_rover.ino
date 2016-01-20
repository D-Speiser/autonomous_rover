#include <Servo.h> 

Servo servo;
const int servoPin = 10;
const int trigPin  = 11;
const int echoPin  = 12;
const int leftMotor []  = {4, 5};
const int rightMotor [] = {6, 7};

long * distances180 = new long [5];
long * distances360 = new long [8];

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
  Serial.begin(9600);
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
  
}

