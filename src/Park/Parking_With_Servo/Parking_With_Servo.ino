#include <Smartcar.h>
#include <Servo.h>
#include <SoftwareSerial.h> // I added this library only for the Bluetooth module.
Odometer encoder;
Gyroscope gyro;
Car car;
Servo myServo;
/*Using the Servo to check the whole back-angle when the car is inside the parking spot.
  right side is angle 200 and back is angle 80
*/
SR04 sensorRight;
SR04 sensorBack;
SR04 sensorFront;
SoftwareSerial Bluetooth(52, 53); // Initialize the Bluetooth module. RX on 52, and TX on 53

/*global variables*/
static boolean startCar = true;
static boolean ServoBack = true;
boolean mover = true;
const int carWidth = 20;
const int carLength = 60;

char input;
int ledPin = 50;
const int TRIGGER_PIN = A1;   // Front-Right Sensor
const int ECHO_PIN = A2;      // Front-Right Sensor
const int TRIGGER_PINB = 6;   // Back Sensor
const int ECHO_PINB = 7;      // Back Sensor
const int TRIGGER_PINF = 44;  // Front Sensor
const int ECHO_PINF = 45;     // Front Sensor
const int encoderPin = 2;     // For Encoder

long duration, durationR, durationF, durationB; // To calculate the duration for each sensor, then we can know the actual distance
int distance, distanceR , distanceF, distanceB; // To calculate each sensor distance

void setup() {
  Bluetooth.begin(9600);                                 // To allow the bluetooth module able to write to our program
  Serial.begin(9600);
  gyro.attach();                                         // Attach the Gyroscope
  myServo.attach(A0);                                    // Attach the Servo with its pin
  encoder.attach(encoderPin);                            // Attach the Encoder with its pin
  sensorRight.attach(TRIGGER_PIN, ECHO_PIN);   // Attach The Front-Right Sensor with its pins
  sensorBack.attach(TRIGGER_PINB, ECHO_PINB);  // Attach The Back Sensor with its pins
  sensorFront.attach(TRIGGER_PINF, ECHO_PINF); // Attach The Front Sensor with its pins
  gyro.begin();                                          // Start Gyroscope counting
  car.begin(encoder, gyro);                              // Start the car with The Encoder and Gyroscope
}

void handleInput() { //handle serial input if there is any
  if (Bluetooth.available()) { // Chech the connection first
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
        if (startCar) { // To keep the findPlace method works until park, we call the method in the while loop
          while (startCar) {
            findPlace(); // Call findPlace method
          }
        }
        else { // otherwise breake this case and call the default case tostop
          input == 's';
        }

        break;

      // In all cases I put the letter "s" as the stop case ((default case))
      default: //if you receive something that you don't know, just stop
        car.setSpeed(0);
        car.setAngle(0);
    }
  }
}

void findPlace() {
  if (distanceRight() == 0 || distanceRight() > 20 ) {
    /*if statement to check the distance in from the Front-Right Sensor
      if the distance == 0 means if there is distance more than the ultrasonic Sensor range  NOT equals to zero */
    encoder.begin();                 // put the encoder in zero, and  Sart count the distance from the encoder (Wheels)
    car.setMotorSpeed(50, 50);       // Set car setSpeed to 50
    while (distanceRight() == 0 || distanceRight() > 20) { // Same condition of if statement's condition , To counting the distance
      // car.setMotorSpeed(40,40);  // Reduce the speed to notes that the car is counting.
      /*
        // Print
        Serial.print(" The Encoder is: ");
        Serial.println(encoder.getDistance());
        Serial.print(" The Distance is: ");
        Serial.println(distanceRight());
      */
      if (encoder.getDistance() > 30) { // if the car goes 30cm and the if statement does not break
        delay(250);
        car.setSpeed(0);                // Stop the car by setting the speed to zero
        delay(300);                     // Wait 500 millisecond
        rotateLeft();                   // Call method rotateLeft, the car should be like ( \ )
        reverseCar();
        correctAngle();
        car.setSpeed(0);                // Stop the car by setting the speed to zero
        delay(200);                     // Wait 200 millisecond
        startCar == false;              // Set the boolean variable startCar to false to break the while loop
        while (true) {             // To stop this method, we use like this while loop
          handleInput();            // Call the main method again
        }
        /*
           Instead to use the while loop we can just put return; to stop the method
        */
      }
    }
  } else {      // Else if the distance not equals to zero or less than 20, keep going ( moving )
    car.setMotorSpeed(50, 50);
  }
}


void rotateLeft() {
  int gDisplacement = gyro.getAngularDisplacement();
  gyro.update();
  car.rotate(-20);
}

void moveWithRotate() {
  car.setSpeed(-40);   // Move backward in speed -40, Note Minus to move backward
  delay(50);           // Wait 50 millisecond
  gyro.update();       // Update the Gyroscope to get the right dimension in X,Y,Z for the car
  car.rotate(-5);      // Rotate to left in -5 degree, Note Minus to turn left
}

void rotateRight() {
  delay(400);      // Wait 4500 millisecond
  gyro.update();   // Update the Gyroscope
  car.rotate(5);   // Rotate to left in 5 degree, Note Minus to turn right

  delay(400);      // Wait 4500 millisecond
  gyro.update();   // Update the Gyroscope
  car.rotate(5);   // Rotate to left in 5 degree, Note Minus to turn right


  delay(400);      // Wait 4500 millisecond
  gyro.update();   // Update the Gyroscope
  car.rotate(5);   // Rotate to left in 5 degree, Note Minus to turn right


  delay(400);      // Wait 4500 millisecond
  gyro.update();   // Update the Gyroscope
  car.rotate(5);   // Rotate to left in 5 degree, Note Minus to turn right

  delay(400);      // Wait 400 millisecond

}

void reverseCar() {
  //todo
  //reverse car 35cm

  int back = 80; //back is angle 80
  int backRight = 200; //back right is angle 200

  myServo.write(back);
  int backSpace = distanceBack() - 2; //additionally, adding a two helps with the significant distance of the sensors on the car
  myServo.write(backRight);
  int backRightSpace = distanceBack() - 2;

  encoder.begin();
  int distanceTraveled = encoder.getDistance();
  //moveCar(-35);
  while (distanceTraveled < 35) {
    car.setMotorSpeed(40, 40);
    distanceTraveled = encoder.getDistance();

    myServo.write(back);
    backSpace = distanceBack() - 2; //additionally, adding a two helps with the significant distance of the sensors on the car
    myServo.write(backRight);
    backRightSpace = distanceBack() - 2;

    if (backRightSpace < 15 || backSpace < 15) {
      return;
    }
    delay(200);
  }
}


void servoCheck() {
  int back = 80; //back is angle 80
  int backRight = 200; //back right is angle 200

  int backSpace = distanceBack() - 2; //additionally, adding a two helps with the significant distance of the sensors on the car
  myServo.write(back);
  int backRightSpace = distanceBack() - 2;
  myServo.write(back);

  while (backSpace > 15 || backRight > 15) {
    car.setMotorSpeed(20, 20);
    moveBackward();
    myServo.write(back);
    backSpace = distanceBack() - 2;
    myServo.write(back);
    backRightSpace = distanceBack() - 2;

  }


}

void callServo() {
  /*This method will call the servo method, then if the servo method is done,
    it will make a double check for the sensorBack distance*/
  while (distanceBack() > 10) { // Check the sensorBack distance if it's greater than 10 run the servo
    if (ServoBack) {
      Serial.println("While");
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
    if (distance && distance < 8) {
      delay(20);
      Serial.print("Check The Distance again checking ");
      Serial.println( distance);
      if (distance && distance < 8) { // A double checking also
        car.go(-2);                     // If it's less than 8, go back 2cm and return to stop the method
        ServoBack == false;
        return;
      }
    }
  }
}
void RunServo() {
  for (int i = 200; i > 40; i -= 5) { // Turn servor from right to left from angle 200 to angle 40
    myServo.write(i);         // Move the servor to angle i
    delay(30);                //  Wait 30 millisecond
    int distance = distanceBack();

    if (distance && distance < 10) {            // Check the distance each time if it less than 10
      delay(20);                              // Wait 20 millisecond
      if (distance && distance < 10) {        // A double checking also
        ServoBack == false;                  // If it's less than 10cm return to stop the method
        return;
      }
    }

  }
  Serial.println("Go Back");
  moveBackward();                            // If the whole objects in the given angle are greater than 10cm then call moveBackward to move backward 7cm

  for (int i = 40; i <= 200; i += 5) {      // Turn servor back from left to right from angle 40 to angle 200, and do same thing
    myServo.write(i);
    delay(30);
    int distance = distanceBack();
    if (distance && distance < 10) {
      delay(20);
      if (distance && distance < 10) {
        ServoBack == false;
        return;
      }
    }
  }
}

int distanceBack() { // To calculate the Back distance faster than getDistance() method
  digitalWrite(TRIGGER_PINB, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINB, LOW);
  durationB = pulseIn(ECHO_PINB, HIGH);
  distanceB = durationB * 0.034 / 2;
  return distanceB;
}

int distanceRight() {  // To calculate the Right distance faster than getDistance() method
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  durationR = pulseIn(ECHO_PIN, HIGH);
  distanceR = durationR * 0.034 / 2;
  return distanceR;
}

int distanceFront() {  // To calculate the Front distance faster than getDistance() method
  digitalWrite(TRIGGER_PINF, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINF, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINF, LOW);
  durationF = pulseIn(ECHO_PINF, HIGH);
  distanceF = durationF * 0.034 / 2;
  return distanceF;
}

void moveBackward() { // Move backword 7cm
  car.go(-7);
}

void moveForward() { // Move forward quarter of the Front distance
  delay(100);
  myServo.write(84); // Set the servo in angle 84
  Serial.println("MOVE FORWARD");
  delay(500);
  int distanceF = distanceFront() / 3;
  delay(200);
  car.go(distanceF);
}

void rotateOnSpot(int targetDegrees) {
  targetDegrees %= 360; //put it on a (-360,360) scale
  if (!targetDegrees) return; //if the target degrees is 0, don't bother doing anything
  /* Let's set opposite speed on each side of the car, so it rotates on spot */
  if (targetDegrees > 0) { //positive value means we should rotate clockwise
    car.setMotorSpeed(40, -40); // left motors spin forward, right motors spin backward
  } else { //rotate counter clockwise
    car.setMotorSpeed(-40, 40); // left motors spin backward, right motors spin forward
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
/*-------------------------------------------------------------------- oliver --------------------------------------------------------------------*/
void loop() {
  park();
}

void moveCar(int distance , int carSpeed) {
  if (carSpeed >= 0 && mover == true) {
    encoder.begin();
    int distanceTraveled = encoder.getDistance();
    car.setSpeed(60);
    while (distanceTraveled < distance) {
      car.setSpeed(carSpeed);
      distanceTraveled = encoder.getDistance();
      //Serial.println(distanceTraveled);
    }
  } else if (carSpeed <= 0 && mover == true) {
    encoder.begin();
    int distanceTraveled = encoder.getDistance();
    car.setSpeed(-60);
    while (distanceTraveled < distance) {
      car.setSpeed(carSpeed);
      distanceTraveled = encoder.getDistance();
      //Serial.println(distanceTraveled);
    }
  }
  mover = false;
  car.setMotorSpeed(0, 0);
}

boolean angleSet = true;

void correctAngle() {
  myServo.write(200);

  if (angleSet == true) {
    /*angle*/
    int front = distanceRight();//front right side
    int back = distanceBack() - 2;/*the varieble, "back", works for the back of the car as well as the back right, additionally, subtracting 2 helps with the significant distance of the sensors on the car*/
    int rightSideCar = front - back;

    Serial.println(rightSideCar);

    while (rightSideCar > 4 || rightSideCar < -2 ) {
      //correcting angle
      if (front > back) {
        car.setSpeed(0);
        car.rotate(1);
        delay(100);
      } else if (back > front) {
        car.setSpeed(0);
        car.rotate(-1);
        delay(100);
      }

      //correcting cars angle
      rightSideCar = front - back;//cars right side
      front = distanceRight();//front right
      back = distanceBack() - 2;//back right (subtracted 2 because of the difference in position)

      if (rightSideCar < 4 || rightSideCar > -2 ) {
        angleSet = false;
      }
    }
  }
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

boolean searching = true;
void search() {
  int rightSpace = distanceRight();
  if (searching == true) {
    car.setMotorSpeed(40, 40);
    rightSpace = distanceRight();

    if (rightSpace == 0 || rightSpace > carWidth) {
      encoder.begin();
      int distanceTraveled = encoder.getDistance();
      while (distanceTraveled < carLength) {
        distanceTraveled = encoder.getDistance();
      }
      car.setSpeed(0);
      searching = false;
    }
  }
}

void park() {
  rotateOnSpot(35);
}

void mainMeth() {
  search();
  rotateLeft();
  park();
  correctAngle();
  handleInput();
  }
