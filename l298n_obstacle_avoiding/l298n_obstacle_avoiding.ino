#include <Servo.h>        //add Servo Motor library            
Servo servo;

#define motora1 13
#define motorb1 12
#define motora2 7
#define motorb2 6
#define enA 11
#define enB 5

int max_speed = 140;

//ultrasonic sensor
#define TRIG A5
#define ECHO A4

int setDistance = 20 ;
int duration = 0;
unsigned int distance = 0;
unsigned int leftDistance;
unsigned int rightDistance;

void setup() {
  Serial.begin(9600);
  pinMode(motora1, OUTPUT);
  pinMode(motorb1, OUTPUT);
  pinMode(motora2, OUTPUT);
  pinMode(motorb2, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  servo.attach(2);
  servo.write(90);
  delay(50);
}

void loop() {
  distance = getDistance();
  Serial.print("dist: ");
  Serial.println(distance);

  //detect the object.
  if (distance <= setDistance) {
    Stop();
    delay(200);
    backward();
    delay(200);
    Stop();
    delay(200);
    lookLeft();
    lookRight();
    if (leftDistance > rightDistance) //if left is less obstructed
    {
      Serial.println("  left");
      left();
      Stop();
    }
    else if (rightDistance > leftDistance) //if right is less obstructed
    {
      Serial.println("  Right");
      right();
      Stop();
    }
    else //if they are equally obstructed
    {
      turnAround();
    }
  }
  forward();  // move forward
}

int getDistance() {
  digitalWrite(TRIG , LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG , HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG , LOW);

  duration = pulseIn(ECHO , HIGH);
  distance = (duration / 2) / 29.1 ;  //29.1 is a speed of sound.
  return distance;
}

int lookLeft () {
  //lock left
  servo.write(144);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  delay(100);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
}

int lookRight() {
  //lock right
  servo.write(36);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  delay(100);
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;

}

void Stop() {
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);
}
void forward() {
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, max_speed);
  analogWrite(enB, max_speed);
}

void backward() {
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, max_speed);
  analogWrite(enB, max_speed);

}
void right() {
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, max_speed);
  analogWrite(enB, max_speed);
  delay(500);
}
void left() {
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, max_speed);
  analogWrite(enB, max_speed);
  delay(500);
}
void turnAround() {
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, max_speed);
  analogWrite(enB, max_speed);
  delay(1000);
}
