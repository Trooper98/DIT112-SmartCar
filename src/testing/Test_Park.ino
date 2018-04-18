#include <Smartcar.h>
Odometer encoder;
Gyroscope gyro;
Car car;
SR04 ultrasonicSensor;
SR04 ultrasonicSensor2;
GP2Y0A21 sideFrontIR;
const int TRIGGER_PIN = 4; // Front-Right Sensor
const int ECHO_PIN = 5; // Front-Right Sensor
const int encoderPin = 2; // For Encoder
const int TRIGGER_PIN2 = 4; // Back Sensor
const int ECHO_PIN2 = 7;//  Back Sensor
void setup() {
  Serial.begin(9600);
  gyro.attach();   // Attach the Gyroscope
  encoder.attach(encoderPin);   // Attach the Encoder with its pin
  ultrasonicSensor.attach(TRIGGER_PIN, ECHO_PIN); // Attach The Front-Right Sensor with its pins
  ultrasonicSensor2.attach(TRIGGER_PIN2, ECHO_PIN2); // Attach The Back Sensor with its pins
  gyro.begin(); // Start Gyroscope counting
  car.begin(encoder, gyro); // Start the car with The Encoder and Gyroscope
}
void loop() {
  // put your main code here, to run repeatedly:
 findPlace();  // A method to find an empty spot
}
void findPlace(){
if(ultrasonicSensor.getDistance() == 0 || ultrasonicSensor.getDistance() > 20 ){ // if statement to check the distance in from the Front-Right Sensor
  //if the distance == 0 means if there is distance more than the ultrasonic Sensor range  NOT equals to zero
  encoder.begin(); //put the encoder in zero, and  Sart count the distance from the encoder (Wheels)
  car.setSpeed(50); // Set car setSpeed to 50
  while(ultrasonicSensor.getDistance() == 0 || ultrasonicSensor.getDistance() > 20){ // Same condition of if statement's condition , To counting the distance
    if(encoder.getDistance() > 38) {  // if the car goes 38cm and the if statement does not break
      car.setSpeed(0);  // Stop the car by setting the speed to zero
      car.setAngle(0);  // Stop any rotate
      makeParkRotate(); // Call makeParkRotate method
            while(true){ // To stop the method, we use like this while loop
            }
           car.setSpeed(0);
              }
      }
    }
  }
  else { // Else if the distance not equals to zero or less than 20, keep going
    car.setSpeed(50);
   }
  }
void makeParkRotate(){
   car.setSpeed(-40); // Move backward in speed -40, Note Minus to move backward
      delay(200); // for 200 millisecond
      gyro.update(); // Update the Gyroscope to get the right dimension in X,Y,Z for the car
      car.rotate(-30); // Rotate to left in -30 degree, Note Minus to turn left
      car.setSpeed(0); // Then srop the car
      delay(500); // Wait for 500 millisecond
      gyro.update(); // Update the Gyroscope again
      car.go(-20); // Move backward in 20 cm
      delay(500); // Wait for 500 millisecond
      makeParkL(); // Call makeParkL method
  }
void makeParkL(){ //  This method does the opposite of  makeParkRotate method
  car.setSpeed(40);// Move forkward in speed 40
  delay(200);  // for 200 millisecond
  gyro.update(); // Update the Gyroscope again
  car.rotate(30); // Rotate to right in 30 degree, Same degree as the left one
}
