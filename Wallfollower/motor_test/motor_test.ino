
#define SPEED 95


//right motor pins
const int in1R = 7;
const int in2R = 8;
const int enR = 9;

//left motor pins
const int in1L = 4;
const int in2L = 5;
const int enL = 3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(in1L,OUTPUT);
  pinMode(in2L,OUTPUT);
  pinMode(in1R,OUTPUT);
  pinMode(in2R,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(enR,OUTPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    analogWrite(enL,SPEED+3);
    analogWrite(enR, SPEED);

}
