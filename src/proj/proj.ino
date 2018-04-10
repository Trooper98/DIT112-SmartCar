#include <Smartcar.h>

//official variables
Car car;
Gyroscope gyro;
Sonar sonar1; //front
const int TRIG_PIN = 5; //sensor's trig pin for back
const int ECHO_PIN = 4; //sensor's echo pin for front
Sonar sonar2; //back
const int TRIG_PIN1 = 7; //sensor's trig pin for back
const int ECHO_PIN1 = 6; //sensor's echo pin for front
GP2Y0A21 infra;
const int frontRightLaser = A0; //use analog pins only
Odometer encoder(100); //let's say there are exactly 100 pulser per meter
const int encoderPin = 2;

//code variables
  boolean park;
  boolean obstruction;
  int right;
  int frontRightS;
  int backS;
  int currDistance;
  int angularDisplacement;
  int count = 0;

void setup() { 
  //attach
  sonar1.attach(TRIG_PIN, ECHO_PIN);
  sonar2.attach(TRIG_PIN1, ECHO_PIN1);
  infra.attach(frontRightLaser);
  encoder.attach(encoderPin);
  gyro.attach();
  
  //begin
  Serial.begin(9600);
  car.begin(encoder, gyro);
  encoder.begin();

  car.setSpeed(70);
}

void loop() {
  
  gyro.update();
  angularDisplacement = gyro.getAngularDisplacement();

  right = infra.getDistance();
  frontRightS = sonar1.getDistance();
  backS = sonar2.getDistance();
  currDistance = encoder.getDistance();
  count++;
  if(count == 300){
    turn(75);
    delay(700);
   }else{
    Serial.println(count);
    }
   
}

void turn(int angle){
  if(angle >= 0 && angle <= 360){
    car.setAngle(angle);
    }
 }


