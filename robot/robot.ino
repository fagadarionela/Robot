#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 10);   //rxPin = 9 txPin = 10;
#define mpin00 5
#define mpin01 6
#define mpin10 3
#define mpin11 11
#define echoPin A2
#define trigPin A1

int b = 0;
float durata = 0;
float distanta = 0;
char dir;
Servo srv;

void setup() {
  mySerial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(mpin00, 0);
  digitalWrite(mpin01, 0);
  digitalWrite(mpin10, 0);
  digitalWrite(mpin11, 0);

  pinMode (mpin00, OUTPUT);
  pinMode (mpin01, OUTPUT);
  pinMode (mpin10, OUTPUT);
  pinMode (mpin11, OUTPUT);

 // pinMode(13, OUTPUT);
}
void StartMotor (int m1, int m2, int forward, int speed)
{
  if (speed == 0) // oprire
  {
    digitalWrite(m1, 0);
    digitalWrite(m2, 0);
  }
  else
  {
    if (forward)
    {
      digitalWrite(m2, 0);
      analogWrite(m1, speed); // folosire PWM
    }
    else
    {
      digitalWrite(m1, 0);
      analogWrite(m2, speed);
    }
  }
}
void delayStopped(int ms)
{
  StartMotor (mpin00, mpin01, 0, 0);
  StartMotor (mpin10, mpin11, 0, 0);
  delay(ms);
}
void playWithServo(char dir, int pin)
{
  srv.attach(pin);
  float poz = srv.read();
  if (dir == 'N') {
    srv.write(90);
  }
  else if (dir == 'V') {
    srv.write(180);
  }
  else if (dir == 'E') {
    srv.write(0);
  }
  delay(1000);
  srv.detach();
}
float calculeazaDistanta() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);
  float durata_l = pulseIn(echoPin, HIGH);
  digitalWrite(2, b);
  b = 1 - b;
  float distanta_l = durata_l / 58.2;
  return distanta_l;
}
void pornesteN() {
  StartMotor (mpin00, mpin01, 1, 128);
  StartMotor (mpin10, mpin11, 0, 128);

  delay (500);
  delayStopped(500);
}
void pornesteS() {
  StartMotor (mpin00, mpin01, 0, 128);
  StartMotor (mpin10, mpin11, 1, 128);

  delay (500);
  delayStopped(500);
}
void pornesteV() {
  StartMotor (mpin00, mpin01, 1, 128);
  StartMotor (mpin10, mpin11, 0, 0);

  delay (500);
  delayStopped(500);
}
void pornesteE() {
  StartMotor (mpin00, mpin01, 1, 0);
  StartMotor (mpin10, mpin11, 0, 128);

  delay (500);
  delayStopped(500);
}
void gasesteCale(char dir) {
  int merge = 0;
  float d = 0;
  if (merge == 0){
  if (dir != 'N') playWithServo('N', 8);
  d = calculeazaDistanta();
  if (d > 20) {
    pornesteN();
    merge = 1;
  }
  }
  if (merge == 0){
  if (dir != 'V') playWithServo('V', 8);
  d = calculeazaDistanta();
  if (d > 20) {
    pornesteV();
    merge = 1;
  }
  }
  if (merge == 0){
  if (dir != 'E' ) playWithServo('E', 8);
  d = calculeazaDistanta();
  if (d > 20) {
    pornesteE();
    merge = 1;
  }}
  if (merge==0){
    pornesteS();
    merge=1;
  }
}
void loop() {

  if (mySerial.available()) {
    dir = mySerial.read();
    if (dir != 'S') {
      playWithServo(dir, 8);
      distanta = calculeazaDistanta();
    }
    if ((distanta <= 20 && distanta >= 0) && dir != 'S') {
      digitalWrite(13, HIGH);
      gasesteCale(dir);
    }
    else {
      digitalWrite(13, LOW);
      if (dir == 'N') {
        pornesteN();
      }
      else if (dir == 'S') {
        pornesteS();
      }
      else if (dir == 'V') {
        pornesteV();
      }
      else if (dir == 'E') {
        pornesteE();
      }
    }
  }
  else {
    distanta = calculeazaDistanta();
    if (distanta <= 20 && distanta >= 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);
    }
  }
}
