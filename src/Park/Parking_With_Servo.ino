#include <Smartcar.h>
#include <Servo.h>
#include <SoftwareSerial.h> // I added this library only for the Bluetooth module.
Odometer encoder;
Gyroscope gyro;
Car car;

Servo myServo; // Using the Servo to check the whole back-angle when the car is inside the parking spot

SR04 ultrasonicSensorRight;
SR04 ultrasonicSensorBack;
SR04 ultrasonicSensorFront;

SoftwareSerial Bluetooth(52, 53); // Initialize the Bluetooth module. RX on 52, and TX on 53

boolean startCar = true;
static boolean ServoBack = true;
static int counterRotate=0;
char input;
int ledPin = 50;
const int TRIGGER_PIN = A1;   // Front-Right Sensor
const int ECHO_PIN = A2;      // Front-Right Sensor

const int TRIGGER_PINB = 6;   // Back Sensor
const int ECHO_PINB = 7;      // Back Sensor

const int TRIGGER_PINF = 44;  // Front Sensor
const int ECHO_PINF = 45;     // Front Sensor
  int des = 200;
const int encoderPin = 2;     // For Encoder

long duration, durationR, durationF, durationB; // To calculate the duration for each sensor, then we can know the actual distance

int distance,distanceR ,distanceF, distanceB;   // To calculate each sensor distance

void setup() {
  Bluetooth.begin(9600);                                 // To allow the bluetooth module able to write to our program
  Serial.begin(9600);
  gyro.attach();                                         // Attach the Gyroscope
  myServo.attach(A0);                                    // Attach the Servo with its pin
  encoder.attach(encoderPin);                            // Attach the Encoder with its pin
  ultrasonicSensorRight.attach(TRIGGER_PIN, ECHO_PIN);   // Attach The Front-Right Sensor with its pins
  ultrasonicSensorBack.attach(TRIGGER_PINB, ECHO_PINB);  // Attach The Back Sensor with its pins
  ultrasonicSensorFront.attach(TRIGGER_PINF, ECHO_PINF); // Attach The Front Sensor with its pins
  gyro.begin();                                          // Start Gyroscope counting
  car.begin(encoder, gyro);                              // Start the car with The Encoder and Gyroscope
}

void loop() {

handleInput(); // Run handleInput which takes inputs from the bluetooth module
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
        while (calculateDistanceFront()> 30){
             Serial.println(calculateDistanceFront() );
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
     while (startCar){ // To keep the findPlace method works until park, we call the method in the while loop
        findPlace(); // Call findPlace method
        }
       startCar=true;
       return;
       break;

      // In all cases I put the letter "s" as the stop case ((default case))
      default: //if you receive something that you don't know, just stop
        car.setSpeed(0);
        car.setAngle(0);
    }
  }
}

void findPlace(){
  if(calculateDistanceRight() == 0 || calculateDistanceRight()> 20 ){
  /*if statement to check the distance in from the Front-Right Sensor
  if the distance == 0 means if there is distance more than the ultrasonic Sensor range  NOT equals to zero */
  encoder.begin();                 // put the encoder in zero, and  Sart count the distance from the encoder (Wheels)
  car.setMotorSpeed(50,50);        // Set car setSpeed to 50
  while(calculateDistanceRight() == 0 || calculateDistanceRight() > 20){ // Same condition of if statement's condition , To counting the distance
    if(encoder.getDistance() > 35) {  // if the car goes 30cm and the if statement does not break
      car.setSpeed(0);                // Stop the car by setting the speed to zero
      car.setAngle(0);                // Stop any rotate
      delay(500);                     // Wait 500 millisecond
      gyro.begin();
      rotateLeft();                   // Call method rotateLeft, the car should be like ( \ )
      callServo();                    // Call method callServo to go backward by checking with the servo and ultrasonicSensorBack;
      rotateRight();                    // Call method rotateRight, the car should be like ( | )
      moveForward();                  // Call method moveForward, to move a little bit forward.
      startCar=false;                // Set the boolean variable startCar to false to break the while loop
      Serial.println("Found");
      car.setSpeed(0);                // Stop the car by setting the speed to zero
      delay(200);                     // Wait 200 millisecond
         return;
          /* while(true){               // To stop this method, we use like this while loop
            handleInput();            // Call the main method again
              } */
      }
    }
  }

  else {      // Else if the distance not equals to zero or less than 20, keep going ( moving )
    car.setMotorSpeed(50,50);
   }
  }

void rotateLeft(){
    int gs = gyro.getAngularDisplacement();
    gyro.update();
    while(gs > 340 || gs == 0) {
        gyro.update();
        gs = gyro.getAngularDisplacement();
        car.rotate(-5);
        car.setSpeed(0);
        delay(400);
        counterRotate=counterRotate+1;
    }
  }

void rotateRight(){
  gyro.update();
  int gs = gyro.getAngularDisplacement();
  while(  counterRotate!=2  ) {
    gyro.update();
    car.rotate(5);
    car.setSpeed(0);
    delay(400);
    counterRotate=counterRotate-1;
  }
}

void callServo(){
  Serial.println("Call Servo");
  /*This method will call the servo method, then if the servo method is done,
  it will make a double check for the ultrasonicSensorBack distance*/
  while(calculateDistanceBack()>10){ // Check the ultrasonicSensorBack distance if it's greater than 10 run the servo
          Serial.println("Call Servo While");
          if (ServoBack){
          Serial.println("While");
            RunServo();
          }
          else if (ServoBack==false){
          return;
          }
             ServoBack == true;
        }
  for(int j=200;j>40;j-=5){ // The second servo's checking
    Serial.println(j);
    myServo.write(j);
    delay(30);
    int distance = calculateDistanceBack();
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

void RunServo(){
Serial.println("RUN SERVO");
  for(int i=200;i>40;i-=5){ // Turn servor from right to left from angle 200 to angle 40
  myServo.write(i);         // Move the servor to angle i
  delay(30);                //  Wait 30 millisecond
  int distance = calculateDistanceBack();
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
  for(int i=40;i<=200;i+=5){                // Turn servor back from left to right from angle 40 to angle 200, and do same thing
  myServo.write(i);
  delay(30);
  int distance = calculateDistanceBack();
       if (distance && distance < 10) {
          delay(20);
          if (distance && distance < 10) {
         ServoBack == false;
         return;
       }
       }
  }
}
int calculateDistanceBack(){  // To calculate the Back distance faster than getDistance() method
  digitalWrite(TRIGGER_PINB, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINB, LOW);
  durationB = pulseIn(ECHO_PINB, HIGH);
  distanceB = durationB*0.034/2;
  return distanceB;
 }

int calculateDistanceRight(){   // To calculate the Right distance faster than getDistance() method
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  durationR = pulseIn(ECHO_PIN, HIGH);
  distanceR = durationR*0.034/2;
  return distanceR;
 }


int calculateDistanceFront(){   // To calculate the Front distance faster than getDistance() method
  digitalWrite(TRIGGER_PINF, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINF, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINF, LOW);
  durationF = pulseIn(ECHO_PINF, HIGH);
  distanceF = durationF*0.034/2;
  return distanceF;
 }

void moveBackward(){ // Move backword 7cm
  car.go(-6);
  }

void moveForward(){ // Move forward quarter of the Front distance
  delay(100);
   myServo.write(84); // Set the servo in angle 84
   Serial.println("MOVE FORWARD");
   delay(500);
    int distanceF = calculateDistanceFront() / 3;
    delay(200);
  car.go(distanceF);
}
