#include <Smartcar.h>
#include <Servo.h>

Car car;
//---------- gyro ----------
const int offset = 32; //this was aquired from the gyroscope callibration example
Gyroscope gyro(offset);

//---------- Sensors ----------
Sonar front; //front
const int TRIG_PIN_F = 4; //sensor's trig pin for front
const int ECHO_PIN_F = 5; //sensor's echo pin for front

Sonar backR; //back rotating
const int TRIG_PIN_B = 6; //sensor's trig pin for back
const int ECHO_PIN_B = 7; //sensor's trig pin for back

GP2Y0A21 infra;
const int INFRA_PIN = A0; //use analog pins only

//---------- Odometer ----------
Odometer encoderRight;
Odometer encoderLeft;
const int ENCODER_RIGHT_PIN = 3;
const int ENCODER_LEFT_PIN = 2;

//---------- Tools ----------
Servo myservo;  // create servo object to control a servo
int const SERVO_PIN = 13;

//---------- Extra ----------
const int mainSpeed = .125; // meters per sec
const int reverseSpeed = .05; // meters per sec

void setup() {
  //attach
  front.attach(TRIG_PIN_F, ECHO_PIN_F);
  backR.attach(TRIG_PIN_B, ECHO_PIN_B);
  infra.attach(INFRA_PIN);
  gyro.attach();
  myservo.attach(SERVO_PIN);
  encoderRight.attach(ENCODER_RIGHT_PIN);
  encoderLeft.attach(ENCODER_LEFT_PIN);

  //begin
  delay(2000);
  Serial.begin(9600);
  gyro.begin(500); //gyro reads every 500ms
  encoderRight.begin();
  encoderLeft.begin();
  car.begin(encoderLeft, encoderRight, gyro);
  car.enableCruiseControl();
  car.setSpeed(mainSpeed); //set the speed to x m/sec
  car.go(1000);
}

void loop() {
  car.updateMotors(); // for cruise control
  int rightDistance = encoderRight.getDistance();
  int leftDistance = encoderLeft.getDistance();
  int distance = leftDistance;
  Serial.println(getRearDistance());
}


//getters
int getFrontDistance() { //distance of front right sensor
  return front.getDistance();
}

int getRearDistance() {
  return infra.getDistance();
}

int getRearRightDistance() {
  return backR.getDistance();
}

//commands
void slowDownStop(){
  for(int i = mainSpeed; i > 0; i--){
    car.setSpeed(i);
    }
  }

//tools
void servoTurn() {
  int pos = 0;
  for (pos = 0; pos <= 75; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 5ms for the servo to reach the position
  }
  for (pos = 75; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 5ms for the servo to reach the position
  }
}


