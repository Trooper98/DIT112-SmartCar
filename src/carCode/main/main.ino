#include <Smartcar.h>
#include <Servo.h>
#include <SoftwareSerial.h> // I added this library only for the Bluetooth module.

byte com = 0;
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

GP2D120 irSensorBack;

const int fSpeed = 70; //70% of the full speed forward
const int bSpeed = -70; //70% of the full speed backward
const int lDegrees = -75; //degrees to turn left
const int rDegrees = 75; //degrees to turn right

const int frontSensor_Trig = 44;  // Front Sensor
const int frontSensor_Echo = 45;  // Front Sensor
const int rightSensor_Trig = A1;  // Front-Right Sensor
const int rightSensor_Echo = A2;  // Front-Right Sensor
const int backSensor_Trig = 6;  // Back Sensor
const int backSensor_Echo = 7;  // Back Sensor
const int encoderPin = 2;     // For Encoder
const int ledPin = 50;
const int IR_PIN = A5;

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
boolean exitPark = true;

const int carSpeed = 40;
const int motorSpeed = 80;
const int carWidth = 20;
const int carLength = 40;
const int servoBack = 80;
const int servoRight = 200;
const int servoPark = 150;

char userInp;

//---------------------------------------------------------- setup() ----------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Bluetooth.begin(9600);                                 // To allow the bluetooth module able to write to our program
  Serial.begin(9600);
  gyro.attach();                                         // Attach the Gyroscope
  myServo.attach(A0);                                    // Attach the Servo with its pin
  encoder.attach(encoderPin);                            // Attach the Encoder with its pin
  irSensorBack.attach(IR_PIN);
  sensorRight.attach(rightSensor_Trig, rightSensor_Echo);   // Attach The Front-Right Sensor with its pins
  sensorBack.attach(backSensor_Trig, backSensor_Echo);  // Attach The Back Sensor with its pins
  sensorFront.attach(frontSensor_Trig, frontSensor_Echo); // Attach The Front Sensor with its pins
  gyro.begin();                                          // Start Gyroscope counting
  car.begin(encoder, gyro);

  Serial.write(0xAA);

  Serial.write(0x37);

  delay(1000);

  Serial.write(0xAA);

  Serial.write(0x21);
}

//__________________________________________________________ loop() __________________________________________________________

void loop() {
  // put your main code here, to run repeatedly:
  remoteControl();
}

//---------------------------------------------------------- remoteControl() ----------------------------------------------------------

void remoteControl() {
  myServo.write(servoBack);
  //__________________________________________________________ voiceCommand() __________________________________________________________
  if (Serial.available()) {
    Serial.println("in voice command");
    com = Serial.read();
    Serial.println(com);
    switch (com) {
      // to do parallel parking
      case 0x11:
        park();
        resetDependancies();
        break;
      // to do perpendicular parking
      case 0x12:
        perpendicularPark();
        resetDependancies();
        break;
      // to exit parking
      case 0x13:
        exitParking();
        resetDependancies();
        break;

      //commands from RPi

      //MoveForward
      case 'w':
        moveCar(10, 50);
        resetDependancies();
        break;

      //TurnLeft
      case 'a':
        car.setSpeed(0);
        gyro.update();
        rotateOnSpot(-25);
        resetDependancies;
        break;

      //TurnRight
      case 'd':
        car.setSpeed(0);
        gyro.update();
        rotateOnSpot(-25);
        resetDependancies;
        break;

      //MoveBackward
      case 's':
        moveCar(10, -50);
        resetDependancies();
        break;

      //Park
      case 'q':
        resetDependancies();
        park();
        break;


    }
  } else  if (Bluetooth.available()) {//======================================= Bluetooth Control() ===================================
    char input =  Bluetooth.read(); //read everything that has been received from the bluetooth
    Serial.println(input);
    switch (input) {

      case 'l': //rotate counter-clockwise going forward
        car.setSpeed(fSpeed);
        car.setAngle(lDegrees);
        break;
      case 'r': //turn clock-wise
        car.setSpeed(fSpeed);
        car.setAngle(rDegrees);
        break;
      case 'f': //go ahead
        if (isFrontFree()) {
          car.setSpeed(fSpeed);
          car.setAngle(0);
        }
        else {
          car.setSpeed(0);
        }
        break;
      case 'b': //go back
        if (isBackFree()) {
          car.setSpeed(bSpeed);
          car.setAngle(0);
        }
        break;
      case 'z':           //To turn the car 90 degrees  to left  then move forward (only for voice command from Google API)
        if (isBackFree()) {
          rotateOnSpot(-90);
          delay(800);
          car.setSpeed(fSpeed);
          car.setAngle(0);
        }
        break;
      case 'c':         //To turn the car 90 degrees  to right then move forward (only for voice command from Google API)
        if (isBackFree()) {
          rotateOnSpot(90);
          delay(800);
          car.setSpeed(fSpeed);
          car.setAngle(0);
        }
        break;
      case 'p': // Find an empty spot to park in it.
        //startCar is a Boolean attribute, we need it to break the loop.
        park();

        resetDependancies();
        myServo.write(servoBack);
        break;

      case 'i': // Find an empty spot to park in it by using IR sensor.
        //startCar is a Boolean attribute, we need it to break the loop.
        parkWithIR();
        resetDependancies();
        myServo.write(servoBack);
        break;

      case 'o': // Exit from the parking spot.
        //exitParking(); // in process by Oliver
        exitParking();
        resetDependancies();
        break;
      // In all cases I put the letter "s" as the stop case ((default case))
      default: //if you receive something that you don't know, just stop
        car.setSpeed(0);
        car.setAngle(0);
    }
  }

  // Check for obstacles while going backward
  if (car.getSpeed() < 0) {
    if (distanceBack() != 0 && distanceBack() < 30) {
      car.setSpeed(0);
      car.setAngle(0);
    }
  }

  // Check for obstacles while going forward
  else if (car.getSpeed() > 0) {
    if (distanceFront() != 0 && distanceFront() < 30) {
      car.setSpeed(0);
      car.setAngle(0);
    }
  }
}


//---------------------------------------------------------- park() ----------------------------------------------------------

void park() {
  Serial.println("in park...");
  if (parking == true) {
    search();
    delay(500);
    enterParkingSpace();
    delay(500);
    // correctAngle();
    delay(500);
    moveForward();
    parking = false;
  }
}
//---------------------------------------------------------- perpendicularPark() ----------------------------------------------------------
void perpendicularPark() {
  if (distanceRight() == 0 || distanceRight() > 20) {
    Serial.println("If");
    encoder.begin();
    car.setMotorSpeed(50, 50);
    while (distanceRight() == 0 || distanceRight() > 20) {
      if (encoder.getDistance() > 12) {
        car.setSpeed(0);
        gyro.begin();
        delay(500);
        car.setSpeed(50);
        moveBackward();
        Serial.println("Before go backward");
        delay(500);
        car.setSpeed(50);
        rotateOnSpot(90);
        car.go(5);
        car.setSpeed(0);
        delay(500);
        Serial.println("After go forward");
        car.setSpeed(0);
        exit(0);
      }
    }
  }
  else {
    Serial.println("Else");
    car.setMotorSpeed(50, 50);
  }
}

//__________________________________________________________ parkWithIR() __________________________________________________________

void parkWithIR() {
  Serial.println("in parkWithIR...");
  if (parking == true) {
    search();
    delay(500);
    enterParkingSpaceUsingIR();
    delay(500);
    //correctAngle();
    delay(500);
    moveForward();
    parking = false;
  }
}

//---------------------------------------------------------- search() ----------------------------------------------------------

void search() {
  Serial.println("in search...");
  // correctAngle();
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

//__________________________________________________________ correctAngle() __________________________________________________________

void correctAngle() {
  Serial.println("in correctAngle...");
  myServo.write(servoRight);

  if (angleSet == true) {
    /*angle*/
    int front = distanceRight();//front right side
    int back = distanceBack() - 2;/*the varieble, "back", works for the back of the car as well as the back right, additionally, subtracting 2 helps with the significant distance of the sensors on the car*/
    int rightSideCar = front - back;

    Serial.println(rightSideCar);

    while (rightSideCar > 2 || rightSideCar < -2 ) {
      //correcting angle
      if (front > back) {
        car.setSpeed(0);
        rotateOnSpot(1);
        delay(300);
      } else if (back > front) {
        car.setSpeed(0);
        rotateOnSpot(-1);
        delay(300);
      }

      //correcting cars angle
      rightSideCar = front - back;//cars right side
      front = distanceRight();//front right
      back = distanceBack() - 2;//back right (subtracted 2 because of the difference in position)

      if (rightSideCar < 2 || rightSideCar > -2 ) {
        angleSet = false;
      }
    }
  }
}

//---------------------------------------------------------- enterParkingSpace() ----------------------------------------------------------

void enterParkingSpace() {
  Serial.println("in enterParkingSpace...");
  int count = 0;
  if (enteringParkSpace == true) {
    delay(500);
    gyro.update();
    rotateParkingCar(-8);
    callServo();
    gyro.update();
    rotateParkingCar(8);
    enteringParkSpace = false;
  }
}

//__________________________________________________________ enterParkingSpaceUsingIR() __________________________________________________________

void enterParkingSpaceUsingIR() {
  Serial.println("in enterParkingSpaceUsingIR...");
  if (enteringParkSpace == true) {
    delay(500);
    gyro.update();
    rotateParkingCar(-8);
    moveBackward();
    checkIR();
    gyro.update();
    rotateParkingCar(8);
    enteringParkSpace = false;
  }
}

//---------------------------------------------------------- rotateParkingCar(int angle) ----------------------------------------------------------

void rotateParkingCar(int angle) {
  delay(900);
  gyro.update();
  rotateOnSpot(angle);
  delay(900);
  gyro.update();
  rotateOnSpot(angle);
  delay(900);
  gyro.update();
  rotateOnSpot(angle);
}

//__________________________________________________________ checkIR() __________________________________________________________

void checkIR() { //Moving backward 6 cm with Infrared Sensor
  int irDistance = irSensorBack.getDistance();
  while (irDistance > 6 ) {   //While the distance on the back side is not equal to 6 cm
    car.setSpeed(-35);       //Move backward with low speed as 35
    irDistance = irSensorBack.getDistance();  // <-- Update Infrared Sensor
  }
  car.setSpeed(0); // Stop the car.
  delay(500); // Wait 20 millisecond
}

//---------------------------------------------------------- reverse() ----------------------------------------------------------

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

//__________________________________________________________ moveCar(int distance , int carSpeed) __________________________________________________________

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

//---------------------------------------------------------- distanceBack() ----------------------------------------------------------

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

//__________________________________________________________ distanceRight() __________________________________________________________

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

//---------------------------------------------------------- distanceFront() ----------------------------------------------------------

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

//__________________________________________________________ jumpStart(boolean direction) __________________________________________________________

void jumpStart(boolean direction) {
  Serial.println("in jumpStart...");
  car.setSpeed(0);
  if (direction) {
    car.setSpeed(60);
  } else {
    car.setSpeed(-60);
  }
}

//---------------------------------------------------------- resetDependancies() ----------------------------------------------------------
void resetDependancies() {
  Serial.println("in reset...");
  angleSet = true;
  mover = true;
  searching = true;
  parking = true;
  enteringParkSpace = true;
  reversing = true;
  exitPark = true;
}

//__________________________________________________________ isBackFree() __________________________________________________________

boolean isBackFree() { //To check the distance in the Backside of the car while car moving
  if (distanceBack() > 0 && distanceBack() > 30) {
    return true;
    car.setSpeed(0);
  }
  return false;
}

//---------------------------------------------------------- isFrontFree() ----------------------------------------------------------

boolean isFrontFree() { //To check the distance in the Frontside of the car while car moving
  if (distanceFront() > 0 && distanceFront() > 30) {
    return true;
    car.setSpeed(0);
  }
  return false;
}

//__________________________________________________________ callServo() __________________________________________________________

void callServo() {
  Serial.println("Call Servo");
  /*This method will call the servo method, then if the servo method is done,
    it will make a double check for the SensorBack distance*/
  moveBackwardTen();

  while (distanceBack() > 12) { // Check the SensorBack distance if it's greater than 10 run the servo
    if (ServoBack) {
      RunServo();
    }
    else if (ServoBack == false) {
      return;
    }
    ServoBack == true;
  }
  for (int j = 200; j > 40; j -= 5) { // The second servo's checking
    Serial.println(j);
    myServo.write(j);
    delay(30);
    int distance = distanceBack();
    if (distance && distance < 12) {
      delay(20);
      Serial.print("Check The Distance again checking ");
      Serial.println( distance);
      if (distance && distance < 12) { // A double checking also
        ServoBack == false;
        return;
      }
    }
  }

}

//---------------------------------------------------------- RunServo() ----------------------------------------------------------

void RunServo() {
  for (int i = 200; i > 40; i -= 5) {             // Turn servor from right to left from angle 200 to angle 40
    myServo.write(i);                               // Move the servor to angle i
    delay(30);                                      //  Wait 30 millisecond
    int distance = distanceBack();
    if (distance && distance < 12) {            // Check the distance each time if it less than 10
      delay(20);                               // Wait 20 millisecond
      if (distance && distance < 12) {        // A double checking also
        ServoBack == false;                  // If it's less than 10cm return to stop the method
        return;
      }
    }
  }
  Serial.println("Move Backward");
  moveBackward();                            // If the whole objects in the given angle are greater than 10cm then call moveBackward to move backward 7cm
  for (int i = 40; i <= 200; i += 5) {       // Turn servor back from left to right from angle 40 to angle 200, and do same thing
    myServo.write(i);
    delay(30);
    int distance = distanceBack();
    if (distance && distance < 12) {
      delay(20);
      if (distance && distance < 12) {
        ServoBack == false;
        return;
      }
    }
  }
}

//__________________________________________________________ moveBackward() __________________________________________________________

void moveBackward() { // Move backword 5cm
  moveCar(5, -50);
  mover = true;
}
void moveForward() { // Move forkword 5cm
  moveCar(5, 50);
  mover = true;
}
void moveBackwardTen() { // Move forkword 10cm
  moveCar(10, -50);
  mover = true;
}

//---------------------------------------------------------- rotateOnSpot(int targetDegrees)() ----------------------------------------------------------

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

void exitParking() {
  Serial.println("in Exit Parking...");
  int front = distanceFront();
  myServo.write(servoBack);
  int back = distanceBack();
  int total = front + back;

  while (exitPark) {
    front = distanceFront();
    back = distanceBack();
    total = front + back;
    if (front > back) {
      Serial.println("leaving parking space...");
      //free to go out
      rotateParkingCar(-8);
      moveCar(carLength, 35);
      rotateParkingCar(8);
      // correctAngle();
      delay(500);
      exitPark = false;
    } else {
      Serial.println("no space...");
      //reverse and then go out
      moveCar(5, -35);
    }
  }
  resetDependancies();
}

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
