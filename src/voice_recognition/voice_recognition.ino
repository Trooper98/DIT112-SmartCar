#include <Smartcar.h>
#include <Servo.h>

byte com = 0;

Odometer encoder;
Gyroscope gyro;
Car car;

Servo myServo; // Using the Servo to check the whole back-angle when the car is inside the parking spot

SR04 ultrasonicSensorRight;
SR04 ultrasonicSensorBack;
SR04 ultrasonicSensorFront;

static boolean startCar = true;
static boolean ServoBack = true;
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

int distance,distanceR ,distanceF, distanceB;   // To calculate each sensor distance
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  gyro.attach();                                         // Attach the Gyroscope
  myServo.attach(A0);                                    // Attach the Servo with its pin
  encoder.attach(encoderPin);                            // Attach the Encoder with its pin
  ultrasonicSensorRight.attach(TRIGGER_PIN, ECHO_PIN);   // Attach The Front-Right Sensor with its pins
  ultrasonicSensorBack.attach(TRIGGER_PINB, ECHO_PINB);  // Attach The Back Sensor with its pins
  ultrasonicSensorFront.attach(TRIGGER_PINF, ECHO_PINF); // Attach The Front Sensor with its pins
  gyro.begin();                                          // Start Gyroscope counting
  
  car.begin(encoder, gyro);
  
   Serial.write(0xAA);

  Serial.write(0x37);

  delay(1000);

  Serial.write(0xAA);

  Serial.write(0x21);
}

void loop() {
  // put your main code here, to run repeatedly:
  voiceCommand();
}

void voiceCommand(){
  while(Serial.available()) {

  com = Serial.read();
Serial.println(com);
  switch(com) {

      case 0x11:    

      car.setSpeed(50);

      break;

      case 0x12:  

      turnRight();

      break;

      case 0x13:  

      turnLeft();

      break;

      case 0x14:  
      
      car.setSpeed(0);

      break;

      case 0x15:  
      
      //startCar is a Boolean attribute, we need it to break the loop.
     if (startCar){ // To keep the findPlace method works until park, we call the method in the while loop
       while (startCar){
        findPlace(); // Call findPlace method
        }
        }
        else { // otherwise breake this case and call the default case tostop
          input=='s';
          }

      break;

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
     // car.setMotorSpeed(40,40);  // Reduce the speed to notes that the car is counting.
     /*
      // Print
     Serial.print(" The Encoder is: ");
     Serial.println(encoder.getDistance());
     Serial.print(" The Distance is: ");
     Serial.println(calculateDistanceRight());
     */
    if(encoder.getDistance() > 30) {  // if the car goes 30cm and the if statement does not break
      car.setSpeed(0);                // Stop the car by setting the speed to zero
      car.setAngle(0);                // Stop any rotate
      delay(500);                     // Wait 500 millisecond
      rotateLeft();                   // Call method rotateLeft, the car should be like ( \ )
      callServo();                    // Call method callServo to go backward by checking with the servo and ultrasonicSensorBack
      rotateRight();                    // Call method rotateRight, the car should be like ( | )
      moveForward();                  // Call method moveForward, to move a little bit forward.
     // makeParkBetter();             // Coming soon (( in process))
      car.setSpeed(0);                // Stop the car by setting the speed to zero
      delay(200);                     // Wait 200 millisecond
      startCar==false;                // Set the boolean variable startCar to false to break the while loop
           while(true){               // To stop this method, we use like this while loop
            voiceCommand();            // Call the main method again
              }
              /*
               * Instead to use the while loop we can just put return; to stop the method
                */
      }
    }
  }

  else {  // Else if the distance not equals to zero or less than 20, keep going ( moving )
    car.setMotorSpeed(50,50);
   }
  }

void callServo(){
  /*This method will call the servo method, then if the servo method is done,
  it will make a double check for the ultrasonicSensorBack distance*/
  while(calculateDistanceBack()>10){ // Check the ultrasonicSensorBack distance if it's greater than 10 run the servo
          if (ServoBack){
          Serial.println("While");
          RunServo();}
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
  Serial.println("Go Back");
  moveBackward();
}

int calculateDistanceBack(){  // To calculate the Back distance faster than getDistance() method
  digitalWrite(TRIGGER_PINB, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PINB, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PINB, LOW);
  durationB = pulseIn(ECHO_PINB, HIGH);
  distanceB = durationB*0.034/2;
  return distance;
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
  car.go(-7);
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

void moveWithRotate(){
  car.setSpeed(-40);   // Move backward in speed -40, Note Minus to move backward
  delay(50);           // Wait 50 millisecond
  gyro.update();       // Update the Gyroscope to get the right dimension in X,Y,Z for the car
  car.rotate(-5);      // Rotate to left in -5 degree, Note Minus to turn left
  }
  
void rotateLeft(){
     moveWithRotate();   // Instead to write four lines of code each time, just call the moveWithRotate method.
     car.setSpeed(0);
     delay(200);         // Wait 200 millisecond
     moveWithRotate();
     car.setSpeed(0);
     delay(200);         // Wait 200 millisecond
     moveWithRotate();
     car.setSpeed(0);
     delay(200);         // Wait 200 millisecond
     moveWithRotate();
     car.setSpeed(0);

     delay(500);         // Wait 500 millisecond
     gyro.update();      // Update the Gyroscope

  }

  void rotateRight(){
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

void turnLeft () {
    car.rotate(-90);
    delay(200);
    car.setSpeed(0);
    }

void turnRight () {
    car.rotate(90);
    delay(200);
    car.setSpeed(0);
    }
    
void makeParkBetter(){ // In process
  int distanceRight = calculateDistanceRight();
  int distanceBack = calculateDistanceBack();
  int distanceFront = calculateDistanceFront();
  if (( distanceRight && distanceRight>10 ) && (distanceBack && distanceBack>10) && (distanceFront && distanceFront>10) ){
      car.setSpeed(-40);
      delay(50);
      gyro.update();
      car.rotate(-5);
      car.rotate(-5);
      car.setSpeed(0);
      delay(200);
      car.go(-5);
      delay(500);
      car.setSpeed(40);
      delay(50);
      gyro.update();
      car.rotate(5);
      car.rotate(5);
      car.setSpeed(0);
      car.go(5);

    }
    /* Serial.print("The distance Right is: ");
    Serial.println(distanceR);
    delay(200);
    Serial.print("The distance Back is: ");
    Serial.println(distanceB);
   delay(200);
    Serial.print("The distance Front is: ");
    Serial.println(distanceF);
    delay(200); */
}


