#include <Servo.h> 

Servo servo;
const int servoPin = 10;
const int trigPin  = 11;
const int echoPin  = 12;

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

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

}

void loop() {
  long * distances = get180Distances();
}

