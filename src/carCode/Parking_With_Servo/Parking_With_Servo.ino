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
const int carWidth = 20;
const int carLength = 60;
const int servoBack = 80;
const int servoRight = 200;
const int servoPark = 150;

char userInp;

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

/ -------------------------------------------------------------------------------------------------- -
void loop() {
  // put your main code here, to run repeatedly:
  remoteControl();

  /*  or   */
  
  //handleInput()
}
//---------------------------------------------------------------------------------------------------



void findPlace() {
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

void search() {
  Serial.println("in search...");
  correctAngle();
  angleSet = true;//set to true for the next angle correctment
  jumpStart(true);
  car.setSpeed(carSpeed);
  int rightSpace = distanceRight();
  while (searching == true) {
    car.updateMotors();
    if (rightSpace == 0 || rightSpace < carWidth) {
      rightSpace = distanceRight();
    } else if (rightSpace == 0 || rightSpace > carWidth) {
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
    car.rotate(-30);
    gyro.update();
    reverse();
    enteringParkSpace = false;
  }
}

void reverse() {
  /* HOW TO USE reverse:
  its a hardcoded method that rotates the car 25 degrees 
  and the procedes to reverse while chacking whether 
  there is space to continue reversing.
  */
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
      car.rotate(25);
      car.setSpeed(0);
      reversing = false;
    }
  }
}

oid jumpStart(boolean direction) {
  
  /* HOW TO USE jumpStart:
  true = forward
  false = backwards
  
  the car needs some "juice" if it starts at low speeds.
  */
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

void remoteControl() {
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
      car.rotate(-25);
      resetDependancies();
      break;
    case 'd'://right
      car.setSpeed(0);
      gyro.update();
      car.rotate(25);
      resetDependancies();
      break;
    case 's'://backwards
      moveCar(10, -50);
      resetDependancies();
      break;
    case 'p'://park
      resetDependancies();
      findPlace();
      break;
  }
}
