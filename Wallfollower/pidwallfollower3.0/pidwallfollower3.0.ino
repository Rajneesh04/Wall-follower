#define SENSOR_MAX 
#define DIST_PROPORTIONAL_CONST 0.034/2 //DEPENDS ON ENVIRONMENT AND SPEED OF SOUND
#define INPUT_DISTANCE 15 //in cm
#define ERROR_DIST 5  // in cm
#define SPEED 95
#define MAX_SENSOR_VALUE 10000
#define MAX_OF_SENSOR 3000
#define MAX_DIFFERNCE 10
#define MAX_TURN_SPEED 10
#define CALIBRATION 3
#define MAX_ALLIGN_ANGLE 5

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
float kp_a = 1;
float kd_a = 0;
//tuning parameters
float kp_d = 1;
float kd_d = 0;
float ki_d = 0;
//distance
float errorD;
float previousErrorD = 0;
float dt = 0.1;

//
float distance1, distance2, currentDistance, integral, derivative, outputD, outputA, angle, previousAngle=0.0, allign_angle=0.0;
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

//////////////////////calculates distance from wall
float current_distance(float read1, float read2){
  float distance = (read1 + read2)/2;  //taking average of two values

  return distance;
  }

//////////////////////which direction to turn
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

//steers the robot to reach at the specified distance
void reach_distance(){
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  currentDistance = current_distance(distance1, distance2);
  errorD = currentDistance - INPUT_DISTANCE;
  derivative = errorD - previousErrorD;
  integral += errorD;
  outputD = kp_d*errorD + ki_d*integral*dt + kd_d*derivative;
  previousErrorD = errorD;
  speedL = SPEED - (int)outputD;
  speedR = SPEED + (int)outputD;
  //right turn 
  if((speedL-speedR)>MAX_DIFFERNCE){
    speedL = SPEED + MAX_DIFFERNCE;
    speedR = SPEED - MAX_DIFFERNCE;
  }else if((speedL-speedR)<(-1)*MAX_DIFFERNCE){
    speedL = SPEED - MAX_DIFFERNCE;
    speedR = SPEED + MAX_DIFFERNCE;
    }
   analogWrite(enL, speedL);
   analogWrite(enR, speedR);
   digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
  
  }

/////////keep the robot alingned to the wall and gives a stable path

void follow_wall(){
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  float angle = distance2-distance1;//if +ve turn left else turn right
  float derivativeA = angle - previousAngle;
  outputA = kp_a*angle + kd_a*derivativeA;
  previousAngle = angle;
  speedL = SPEED + CALIBRATION - outputA;
  speedR = SPEED + outputA;
  if((speedL-speedR)>MAX_TURN_SPEED){
    speedL = SPEED + CALIBRATION + MAX_TURN_SPEED;
    speedR = SPEED - MAX_TURN_SPEED;
    }else if((speedL-speedR)>MAX_TURN_SPEED){
      speedL = SPEED + CALIBRATION - MAX_TURN_SPEED;
      speedR = SPEED + MAX_TURN_SPEED;
      }
  analogWrite(enL, speedL);
  analogWrite(enR, speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
  
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
  region = check_region(distance1, distance2);
  allign_angle = abs(distance2 - distance1);
  Serial.print("distance1:");
  Serial.println(distance1);
  Serial.print("distance2:");
  Serial.println(distance2);
  Serial.print("region:");
  Serial.println(region);
  if(region==0){
    follow_wall();
    }
   else{
    if(allign_angle>MAX_ALLIGN_ANGLE){
      follow_wall();
      }else{
        reach_distance();
      }
    
    }

}
