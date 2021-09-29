#include<Servo.h>
//motors
#define motora1 13
#define motorb1 12
#define motora2 7
#define motorb2 6
#define enA 11
#define enB 5

//ir sensors
#define RirSensor A0
#define LirSensor A1

//ultrasonic sensor
#define TRIG A5
#define ECHO A4

int duration = 0;
unsigned int distance = 0;

Servo servo;
int angle1 = 0;

int Max_speed = 110;
int righttyre = 90;
int lefttyre  = 90;

void setup() {
  // put your setup code here, to run once:
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

  for (angle1 = 90; angle1 <= 180; angle1 += 1) {
    servo.write(angle1);
    delay(15);
  } for (angle1 = 180; angle1 >= 0; angle1 -= 1) {
    servo.write(angle1);
    delay(15);
  } for (angle1 = 0; angle1 <= 90; angle1 += 1) {
    servo.write(angle1);
    delay(15);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  distance = getDistance();
  Serial.print("dist: ");
  Serial.println(distance);

  int LEFT_SENSOR = digitalRead(LirSensor);
  int RIGHT_SENSOR = digitalRead(RirSensor);

  Serial.print("leftsenor: ");
  Serial.print(LEFT_SENSOR);
  Serial.print("   ");
  Serial.print("RIghtsenor: ");
  Serial.print(RIGHT_SENSOR);

  if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 1) && (distance >= 10 && distance <= 30)) {
    forward();
    Serial.println("Forward");
  } else if ((RIGHT_SENSOR == 0) && (LEFT_SENSOR == 0) && distance <= 10) {
    backward();
    Serial.println("backward");
  }else if ((RIGHT_SENSOR == 0) && (LEFT_SENSOR == 1)) {
    left();
    delay(200);
    Serial.println("left");
  } else if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 0)) {
    right();
    delay(200);
    Serial.println("right");
  } else if ((RIGHT_SENSOR == 1) && (LEFT_SENSOR == 1)) {
    Stop();
    Serial.println("stop");
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
void forward()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, Max_speed);
  analogWrite(enB, Max_speed);
}

void backward()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, Max_speed);
  analogWrite(enB, Max_speed);
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
void Stop()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);
}
