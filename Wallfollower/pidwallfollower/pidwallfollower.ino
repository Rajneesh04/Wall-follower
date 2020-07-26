

#define SENSOR_MAX 
#define DIST_PROPORTIONAL_CONST 0.034/2 //DEPENDS ON ENVIRONMENT AND SPEED OF SOUND
#define INPUT_DISTANCE 15 //in cm
#define ERROR_DIST 5  // in cm
#define SPEED 80
#define MAX_SENSOR_VALUE 10000
#define MAX_OF_SENSOR 3000
#define CLIBRATION 3
//sensor 1 pins
const int trigPin1 = 10;
const int echoPin1 = 11;

//sensor 2 pins
const int trigPin2 = 12;
const int echoPin2 = 13;

//right motor pins
const int in1R = 7;
const int in2R = 8;
const int enR = 9;

//left motor pins
const int in1L = 4;
const int in2L = 5;
const int enL = 3;

//pid
//tuning parameters
float kp = 2;
float kd = 0;
float ki = 0;
//distance
float errorD;
float previousErrorD = 0;
float dt = 0.1;

//
float distance1, distance2, currentDistance, integral, derivative, output ;
int region, pidDist, speedL, speedR, distance;

//reading sensor
float sensor_output(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  Serial.print("duration:");
  Serial.println(duration);
  if(duration==0.0||duration>=MAX_SENSOR_VALUE){
    duration = MAX_OF_SENSOR;
    }
    

  float distance = duration*DIST_PROPORTIONAL_CONST;
   
  return distance; 
  }

float current_distance(float read1, float read2){
  float distance = (read1 + read2)/2;  //taking average of two values

  return distance;
  }

int check_region(float read1, float read2){
  distance = current_distance(read1, read2);
  if(abs(distance-INPUT_DISTANCE)>ERROR_DIST){
    if(distance > INPUT_DISTANCE){
        return -1; //for left movement
      }else{
        return 1; //for right movement
        }
    }else{
      return 0; //inside region
      }
  }

 float pid_distance(float read1, float read2){
   currentDistance = current_distance(read1, read2);
   errorD = currentDistance - INPUT_DISTANCE;
   integral = integral + errorD*dt;
   derivative = (errorD - previousErrorD)/dt;
   output = kp*errorD + ki*integral + kd*derivative;
   previousErrorD  = errorD;

  return output;
  }
  
void setup() {
  Serial.begin(9600);
  pinMode(in1L,OUTPUT);
  pinMode(in2L,OUTPUT);
  pinMode(in1R,OUTPUT);
  pinMode(in2R,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(enR,OUTPUT);                          
  
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
}

void loop() {
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  Serial.print("distance1:");
  Serial.println(distance1);
  Serial.print("distance2:");
  Serial.println(distance2);
  region = check_region(distance1, distance2);
  Serial.print("region:");
  Serial.println(region);
  pidDist = ceil(pid_distance(distance1, distance2));
  Serial.print("pidDist:");
  Serial.println(pidDist);
  speedL = SPEED - pidDist;
  speedR = SPEED + pidDist; 
  Serial.print("LEFT:");
  Serial.println(speedL);
  Serial.print("RIGHT:");
  Serial.println(speedR);
  if(speedR>255){
    speedR = 255;
  }
  if(speedR<0){
    speedR = 0; 
   }
  if(speedL>200){
    speedL = 200;
  }
  if(speedL<50){
    speedL = 50; 
   }

  if(region == 0){
    analogWrite(enL, SPEED+4);
    analogWrite(enR, SPEED);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    }
   else{
    analogWrite(enL, speedL+ CALIBRATION);
    analogWrite(enR, speedR);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    }
 //delay(1000);
 Serial.println("");
}
