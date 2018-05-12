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
boolean rp = true;

const int carWidth = 20;
const int carLength = 60;
const int servoBack = 80;
const int servoRight = 200;

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
  remoteControl();
}
//---------------------------------------------------------------------------------------------------

void remoteControl() {
  Serial.println("in remote...");
  userInp = Serial.read();
  switch (userInp) {
    case 'w'://forward
      break;
    case 'a'://left
      break;
    case 'd'://right
      break;
    case 's'://backwards
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
      rotatePark();
      break;
  }
}

void park() {
  Serial.println("in park...");
  if (parking == true) {
    search();
    delay(500);
    rotatePark();
    delay(500);
    //moveCar(35, -30);
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

void search() {
  Serial.println("in search...");
  correctAngle();
  angleSet = true;//set to true for the next angle correctment
  jumpStart(true);
  car.setSpeed(30);
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

void rotatePark() {
  Serial.println("in rotatePark...");
  int counter = 0; //controls forloop
  int gDisplacement = gyro.getAngularDisplacement();
  gyro.update();

  if (rp == true) {
    for (int i = 0; i < 4; i++) {
      jumpStart(false);
      car.setSpeed(-30);
      gyro.update();
      car.rotate(-4);
      counter ++;
      delay(500);
    }
    if (counter <= 4) {
      car.setSpeed(0);
      rp = false;
      moveCar(25, -40);
    }
  }
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

void travel(int angle) {
  int gyroDisplacement = gyro.getAngularDisplacement();
  gyro.update();

  //rotate by angle and travel
}

void resetDependancies() {
  Serial.println("in reset...");
  angleSet = true;
  mover = true;
  searching = true;
  parking = true;
  rp = true;
}
