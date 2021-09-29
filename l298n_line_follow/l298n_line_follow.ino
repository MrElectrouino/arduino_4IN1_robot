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

int maxSpeed = 100; 

int turnSpeed = 80;

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
}

void loop() {
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
    Serial.println("Stop");
    Stop();
  }
}

void forward()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, maxSpeed);
  analogWrite(enB, maxSpeed);
}

void right()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, HIGH);
  digitalWrite(motora2, HIGH);
  digitalWrite(motorb2, LOW);

  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}

void left()
{
  digitalWrite(motora1, HIGH);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, HIGH);

  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}
void Stop()
{
  digitalWrite(motora1, LOW);
  digitalWrite(motorb1, LOW);
  digitalWrite(motora2, LOW);
  digitalWrite(motorb2, LOW);
}
