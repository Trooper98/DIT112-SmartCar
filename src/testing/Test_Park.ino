#include <Smartcar.h>
Odometer encoder;
Gyroscope gyro;
Car car;
SR04 ultrasonicSensor;
SR04 ultrasonicSensor2;
GP2Y0A21 sideFrontIR;
const int TRIGGER_PIN = 4;
const int ECHO_PIN = 5;
const int encoderPin = 2;
const int TRIGGER_PIN2 = 4;
const int ECHO_PIN2 = 7;// Pack one
void setup() {
  Serial.begin(9600);
  gyro.attach();
  encoder.attach(encoderPin);
  ultrasonicSensor.attach(TRIGGER_PIN, ECHO_PIN);
  ultrasonicSensor2.attach(TRIGGER_PIN2, ECHO_PIN2);
  gyro.begin();
  car.begin(encoder, gyro);
}
void loop() {
  // put your main code here, to run repeatedly:
 findPlace();
}
void findPlace(){
if(ultrasonicSensor.getDistance() == 0 || ultrasonicSensor.getDistance() > 20 ){
  encoder.begin();
  car.setSpeed(50);
  while(ultrasonicSensor.getDistance() == 0 || ultrasonicSensor.getDistance() > 20){
    if(encoder.getDistance() > 38) {
      car.setSpeed(0);
      car.setAngle(0);
      makeParkRotate();
            while(true){
           car.setSpeed(0);
              }
      }
    }
  }
  else {
    car.setSpeed(90);
   }
  }
void makeParkRotate(){
   car.setSpeed(-40);
      delay(200);
      gyro.update();
      car.rotate(-30);
      car.setSpeed(0);
      delay(500);
      gyro.update();
      car.go(-40);
      delay(500);
      makeParkL();
  }
void makeParkL(){
  car.setSpeed(40);
  delay(200);
   gyro.update();
      car.rotate(30);
}



Add CommentCollapse 
