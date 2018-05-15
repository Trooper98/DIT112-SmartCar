#include <Smartcar.h>
#include <Servo.h>
#include <SoftwareSerial.h> // I added this library only for the Bluetooth module.
Odometer encoder;
Gyroscope gyro;
Car car;
SR04 sensorRight;
SR04 sensorBack;
SR04 sensorFront;
SoftwareSerial Bluetooth(52, 53); // Initialize the Bluetooth module. RX on 52, and TX on 53
Servo myServo;
/*Using the Servo to check the whole back-angle when the car is inside the parking spot.
  right side is angle 200 and back is angle 80
*/
const int frontSensor_Trig = 44;  // Front Sensor
const int frontSensor_Echo = 45;  // Front Sensor
const int rightSensor_Trig = A1;  // Front-Right Sensor
const int rightSensor_Echo = A2;  // Front-Right Sensor
const int backSensor_Trig = 6;  // Back Sensor
const int backSensor_Echo = 7;  // Back Sensor
const int encoderPin = 2;     // For Encoder
const int ledPin = 50;

/*global variables*/
long duration, durationR, durationF, durationB; // To calculate the duration for each sensor, then we can know the actual distance
int distance, distanceR , distanceF, distanceB; // To calculate each sensor distance

static boolean startCar = true;
static boolean ServoBack = true;

//method dependancies *very important*
boolean angleSet = true;
boolean mover = true;
boolean searching = true;
boolean parking = true;
boolean enteringParkSpace = true;
boolean reversing = true;

const int carSpeed = 40;
const int motorSpeed = 80;
const int carWidth = 20;
const int carLength = 40;
const int servoBack = 80;
const int servoRight = 200;
const int servoPark = 150;

char userInp;

//

void setup() {
  // put your setup code here, to run once:
  Bluetooth.begin(9600);                                 // To allow the bluetooth module able to write to our program
  Serial.begin(9600);
  gyro.attach();                                         // Attach the Gyroscope
  myServo.attach(A0);                                    // Attach the Servo with its pin
  encoder.attach(encoderPin);                            // Attach the Encoder with its pin
  sensorRight.attach(rightSensor_Trig, rightSensor_Echo);   // Attach The Front-Right Sensor with its pins
  sensorBack.attach(backSensor_Trig, backSensor_Echo);  // Attach The Back Sensor with its pins
  sensorFront.attach(frontSensor_Trig, frontSensor_Echo); // Attach The Front Sensor with its pins
  gyro.begin();                                          // Start Gyroscope counting
  car.begin(encoder, gyro);
}
//---------------------------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
  //remoteControl();
  park();
}
//---------------------------------------------------------------------------------------------------

void remoteControl() {
  if (Serial.available()) {
    Serial.println("in remote...");
    userInp = Serial.read();
    switch (userInp) {
      case 'w'://forward
        moveCar(10, 50);
        resetDependancies();
        break;
      case 'a'://left
        car.setSpeed(0);
        gyro.update();
        rotateOnSpot(-25);
        resetDependancies();
        break;
      case 'd'://right
        car.setSpeed(0);
        gyro.update();
        rotateOnSpot(25);
        resetDependancies();
        break;
      case 's'://backwards
        moveCar(10, -50);
        resetDependancies();
        break;
      case 'p'://park
        resetDependancies();
        park();
        break;
      case 'r'://rotate car in park mode
        resetDependancies();
        search();
        break;
      case 'e':
        resetDependancies();
        enterParkingSpace();
        break;
      case 'c':
        rotateOnSpot(-20);
        break;
    }
  } else if (Bluetooth.available()) {
    char input =  Bluetooth.read(); //read everything that has been received from the bluetooth
    Serial.println(input);
    switch (input) {

      case 'l': //rotate counter-clockwise going forward
        car.setSpeed(70);
        car.setAngle(-75);
        break;
      case 'r': //turn clock-wise
        car.setSpeed(70);
        car.setAngle(75);
        break;
      case 'f': //go ahead
        car.setSpeed(75);
        car.setAngle(0);
        break;
      case 'w': //go ahead until if there is any object in 30cm stop
        while (distanceFront() > 30) {
          Serial.println(distanceFront() );
          car.setSpeed(75);
          car.setAngle(0);
        }
        break;
      case 'b': //go back
        car.setSpeed(-70);
        car.setAngle(0);
        break;
      case 'p': // Find an empty spot to park in it.
        //startCar is a Boolean attribute, we need it to break the loop.
        park();
        enterParkingSpace();
        break;
      // In all cases I put the letter "s" as the stop case ((default case))
      default: //if you receive something that you don't know, just stop
        car.setSpeed(0);
        car.setAngle(0);
    }
  }
}

void park() {
  Serial.println("in park...");
  if (parking == true) {
    search();
    delay(500);
    enterParkingSpace();
    delay(500);
    correctAngle();
    delay(500);
    parking = false;
  }
}

void search() {
  Serial.println("in search...");
  //correctAngle();
  int rightSpace;
  int rightLenght;
  while (searching == true) {
    //angleSet = true;//set to true for the next angle correctment
    car.setSpeed(carSpeed);
    rightSpace = distanceRight();
    if (rightSpace == 0 || rightSpace > carWidth) {
      //if there is space left
      Serial.println("found right space");
      car.setSpeed(carSpeed - 10);
      encoder.begin();
      while ((rightSpace == 0 || rightSpace > carWidth) && searching == true) {//while right is free and checking length
        rightSpace = distanceRight();
        //while there is space
        Serial.println("checking right length");
        rightLenght = encoder.getDistance();
        if (rightLenght > carLength) {
          Serial.println("FOUND");
          //check length... if length !enough exit
          searching = false;
          car.setSpeed(0);
        }
      }
    } else {
      Serial.println("NOTHING");
      car.setSpeed(carSpeed);
    }
  }
}

void correctAngle() {
  Serial.println("in correctAngle...");
  myServo.write(servoRight);

  if (angleSet == true) {
    /*angle*/
    int front = distanceRight();//front right side
    int back = distanceBack() - 2;/*the varieble, "back", works for the back of the car as well as the back right, additionally, subtracting 2 helps with the significant distance of the sensors on the car*/
    int rightSideCar = front - back;

    Serial.println(rightSideCar);

    while (rightSideCar > 3 || rightSideCar < -2 ) {
      //correcting angle
      if (front > back) {
        car.setSpeed(0);
        rotateOnSpot(1);
        delay(100);
      }
      /*else if (back > front) {
        car.setSpeed(0);
        rotateOnSpot(-1);
        delay(100);
      }*/

      //correcting cars angle
      rightSideCar = front - back;//cars right side
      front = distanceRight();//front right
      back = distanceBack() - 2;//back right (subtracted 2 because of the difference in position)

      if (rightSideCar < 3 || rightSideCar > -2 ) {
        angleSet = false;
      }
    }
  }
}

void enterParkingSpace() {
  Serial.println("in enterParkingSpace...");
  int count = 0;
  if (enteringParkSpace == true) {
    delay(500);
    rotateOnSpot(-20);
    gyro.update();
    reverse();
    delay(500);
    rotateOnSpot(5);
    delay(100);
    rotateOnSpot(5);
    delay(100);
    rotateOnSpot(5);
    delay(100);
    rotateOnSpot(5);
    enteringParkSpace = false;
  }
}

void reverse() {
  myServo.write(servoPark);
  int back = distanceBack();
  Serial.print(back);
  while (reversing) {
    back = distanceBack();
    if (back >= 10) {
      moveCar(5, -carSpeed);
      mover = true;
      delay(500);
    } else {
      car.setSpeed(0);
      reversing = false;
    }
  }
}

void moveCar(int distance , int carSpeed) {
  Serial.println("in moveCar...");
  if (carSpeed >= 0 && mover == true) {
    encoder.begin();
    int distanceTraveled = encoder.getDistance();
    jumpStart(true);
    while (distanceTraveled < distance) {
      car.setSpeed(carSpeed);
      distanceTraveled = encoder.getDistance();
      //Serial.println(distanceTraveled);
    }
  } else if (carSpeed <= 0 && mover == true) {
    encoder.begin();
    int distanceTraveled = encoder.getDistance();
    jumpStart(false);
    while (distanceTraveled < distance) {
      car.setSpeed(carSpeed);
      distanceTraveled = encoder.getDistance();
      //Serial.println(distanceTraveled);
    }
  }
  mover = false;
  car.setMotorSpeed(0, 0);
}

int distanceBack() { // To calculate the Back distance faster than getDistance() method
  digitalWrite(backSensor_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(backSensor_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(backSensor_Trig, LOW);
  durationB = pulseIn(backSensor_Echo, HIGH);
  distanceB = durationB * 0.034 / 2;
  return distanceB;
}

int distanceRight() {  // To calculate the Right distance faster than getDistance() method
  digitalWrite(rightSensor_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(rightSensor_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(rightSensor_Trig, LOW);
  durationR = pulseIn(rightSensor_Echo, HIGH);
  distanceR = durationR * 0.034 / 2;
  return distanceR;
}

int distanceFront() {  // To calculate the Front distance faster than getDistance() method
  digitalWrite(frontSensor_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(frontSensor_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontSensor_Trig, LOW);
  durationF = pulseIn(frontSensor_Echo, HIGH);
  distanceF = durationF * 0.034 / 2;
  return distanceF;
}

void jumpStart(boolean direction) {
  Serial.println("in jumpStart...");
  car.setSpeed(0);
  if (direction) {
    car.setSpeed(60);
  } else {
    car.setSpeed(-60);
  }
}

void resetDependancies() {
  Serial.println("in reset...");
  angleSet = true;
  mover = true;
  searching = true;
  parking = true;
  enteringParkSpace = true;
  reversing = true;
}

void rotateOnSpot(int targetDegrees) {
  targetDegrees %= 360; //put it on a (-360,360) scale
  if (!targetDegrees) return; //if the target degrees is 0, don't bother doing anything
  /* Let's set opposite speed on each side of the car, so it rotates on spot */
  if (targetDegrees > 0) { //positive value means we should rotate clockwise
    car.setMotorSpeed(motorSpeed, -motorSpeed); // left motors spin forward, right motors spin backward
  } else { //rotate counter clockwise
    car.setMotorSpeed(-motorSpeed, motorSpeed); // left motors spin backward, right motors spin forward
  }
  unsigned int initialHeading = gyro.getAngularDisplacement(); //the initial heading we'll use as offset to calculate the absolute displacement
  int degreesTurnedSoFar = 0; //this variable will hold the absolute displacement from the beginning of the rotation
  while (abs(degreesTurnedSoFar) < abs(targetDegrees)) { //while absolute displacement hasn't reached the (absolute) target, keep turning
    gyro.update(); //update to integrate the latest heading sensor readings
    int currentHeading = gyro.getAngularDisplacement(); //in the scale of 0 to 360
    if ((targetDegrees < 0) && (currentHeading > initialHeading)) { //if we are turning left and the current heading is larger than the
      //initial one (e.g. started at 10 degrees and now we are at 350), we need to substract 360, so to eventually get a signed
      currentHeading -= 360; //displacement from the initial heading (-20)
    } else if ((targetDegrees > 0) && (currentHeading < initialHeading)) { //if we are turning right and the heading is smaller than the
      //initial one (e.g. started at 350 degrees and now we are at 20), so to get a signed displacement (+30)
      currentHeading += 360;
    }
    degreesTurnedSoFar = initialHeading - currentHeading; //degrees turned so far is initial heading minus current (initial heading
    //is at least 0 and at most 360. To handle the "edge" cases we substracted or added 360 to currentHeading)
  }
  car.stop(); //we have reached the target, so stop the car
}

//Testing--------------------------------------------------------------------------------------------------
void correctPlacement() {
  /*front back space*/
  int front = distanceFront();
  myServo.write(80);
  int back = distanceBack();
  int totalSpace = front + back;
  int range = (totalSpace / 2) + 3; //the front space to back space, range accepted
  int distanceToMove; //used within while loop

  /*int scale = 0;
    while (true) {
    front = distanceFront();
    back = distanceBack();
    totalSpace = front + back;
    range = (totalSpace / 2) + 3;
    if (front > range || back > range) {
      scale = 0;
      Serial.println(scale);
    } else if (front < range || back < range) {
      scale = 1;
      Serial.println(scale);
    }
    }*/


  while (front > range || back > range) {
    front = distanceFront();
    back = distanceBack();
    totalSpace = front + back;
    range = (totalSpace / 2) + 3;
    if (front > back) {
      //move forward
      distanceToMove = front - back;
      //moveCar(distanceToMove);
    } else if (front < back) {
      //move backwards
      distanceToMove = back - front;
      //moveCar(-1 * distanceToMove);
    }
    front = distanceFront();
    back = distanceBack();
    if (front < range || back < range) {
      return;
    }
  }
}
void gyroControl(){
  gyro.begin();
  int gryoD = gyro.getAngularDisplacement();
  }
