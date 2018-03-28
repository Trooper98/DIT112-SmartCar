#include <Smartcar.h>
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

void setup() { 
  //attach
  sonar1.attach(TRIG_PIN, ECHO_PIN);
  sonar2.attach(TRIG_PIN1, ECHO_PIN1);
  infra.attach(frontRightLaser);
  encoder.attach(encoderPin);
  gyro.attach();
  
  //begin
  Serial.begin(9600);
  car.begin(gyro);
  encoder.begin();


  //code
  boolean go = true;
  int right = infra.getDistance();
  int frontRightS = sonar1.getDistance();
  int backS = sonar2.getDistance();
  int currDistance = encoder.getDistance();
  car.setMotorSpeed(60,60);
}

void loop() {
  // i put code in set up because it doesn't work properly in loop 
    
    /*boolean go = true;
  int right = infra.getDistance();
  int frontRightSonar = sonar1.getDistance();
  int backSonar = sonar2.getDistance();
  int currDistance = encoder.getDistance();
  
    while(go){
    right = infra.getDistance();
    frontRightSonar = sonar1.getDistance();
    backSonar = sonar2.getDistance();
    currDistance = encoder.getDistance();
      gyro.update();
        if((right <= 15 && right >0)){
          go = false;
        }else{
           car.setMotorSpeed(50, 50);
           }
           delay(400);
      }*/
    //Serial.println(gyro.getAngularDisplacement());
    //Serial.println(frontRightSonar);
    //Serial.println(backSonar);
    //Serial.println(right);

  boolean go = true;
  int right = infra.getDistance();
  int frontRightSonar = sonar1.getDistance();
  int backSonar = sonar2.getDistance();
  int currDistance = encoder.getDistance();

  Serial.println(frontRightSonar);
  if(frontRightSonar <= 35 && frontRightSonar > 0){ 
    car.setMotorSpeed(0,0);
    delay(1000);
    car.setMotorSpeed(-50, -20);
    delay(10000);
    }
    
}


