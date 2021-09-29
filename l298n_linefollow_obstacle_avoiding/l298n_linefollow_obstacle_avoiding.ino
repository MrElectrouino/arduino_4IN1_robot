#include<Servo.h>
//motors
#define motora1 13
#define motorb1 12
#define motora2 7
#define motorb2 6
#define enA 11
#define enB 5

//ir sensors
#define LirSensor 3
#define RirSensor 4

//For servo motor
Servo servo;

//ultrasonic sensor
#define TRIG A5
#define ECHO A4

int duration = 0;
unsigned int distance = 0;
unsigned int leftDistance;
unsigned int rightDistance;
boolean object;

/*change this*/
///////////////////////////////
int servo_angle = 90;        //
//
//motor speed controller     //
int Motor_speed = 90;        // speed of motor
int LeftRotationSpeed = 80;  // Left turn Speed
int RightRotationSpeed = 80; // Right turn Speed
int righttyre = 80;          //
int lefttyre = 80;           //
//
//Ultrasonic sensor          //
int setDistance = 15;        // Object detect distance
//When object is detected, robot should move either left or right. so you need to adjust the turn.
int turn1    = 500;          //
int turn2    = 800;          //
int turn3    = 1000;         //
///////////////////////////////

void setup() {
  Serial.begin(9600);
  pinMode(motora1, OUTPUT);
  pinMode(motorb1, OUTPUT);
  pinMode(motora2, OUTPUT);
  pinMode(motorb2, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(LirSensor, INPUT);
  pinMode(RirSensor, INPUT);

  pinMode(TRIG , OUTPUT);
  pinMode(ECHO , INPUT);

  servo.attach(2);
  servo.write(90);

}

void loop() {
  distance = getDistance();
  Serial.print("dist: ");
  Serial.println(distance);
  //detect the object.
  if (distance <= setDistance) {
    Stop();
    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      //right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100);
  } else {
    //for Line follow
    int LEFT_SENSOR = digitalRead(LirSensor);
    int RIGHT_SENSOR = digitalRead(RirSensor);

    Serial.print("leftsenor: ");
    Serial.print(LEFT_SENSOR);
    Serial.print("   ");
    Serial.print("RIghtsenor: ");
    Serial.print(RIGHT_SENSOR);

    if (RIGHT_SENSOR == 0 && LEFT_SENSOR == 0) {
      forward(); //FORWARD
      Serial.println("   forword");
    }
    else if (RIGHT_SENSOR == 1 && LEFT_SENSOR == 0) {
      right(); //Move Right
      Serial.println("   right turn");
    }
    else if (RIGHT_SENSOR == 0 && LEFT_SENSOR == 1) {
      left(); //Move Left
      Serial.println("   left turn");
    }
    else if (RIGHT_SENSOR == 1 && LEFT_SENSOR == 1) {
      //Serial.println("stop then grab an object");
      Stop();
    }
  }
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
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {
  //lock right
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}
void turn() {
  if (object == false) {
    Serial.println("turn Right");
    Stop();
    delay(100);
    singleRight();
    delay(turn1);
    forward();
    delay(turn2);
    singleLeft();
    delay(turn3);
    if (digitalRead(LirSensor) == 1) {
      loop();
    } else {
      forward();
    }
  } else {
    Serial.println("turn left");
    Stop();
    delay(100);
    singleLeft();
    delay(400);
    forward();
    delay(900);
    singleRight();
    delay(500);
    if (digitalRead(LirSensor) == 1) {
    Stop();
    delay(100);
      loop();
    } else {
      forward();
    }
  }
}


void forward()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void right()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, lefttyre);
  analogWrite(enB, righttyre);
}

void left()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, lefttyre);
  analogWrite(enB, righttyre);
}

void singleLeft()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, Motor_speed);
  analogWrite(enB, 0);
}

void singleRight()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, Motor_speed);
}
void Stop()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);
}
