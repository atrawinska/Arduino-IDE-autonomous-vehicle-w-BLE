
#include <ESP32Servo.h>

static const int servoPin = 28;


Servo myservo;
int pos = 90; //changable value of the current position
const int rightMax = 97; //maximum angle for turning right
const int leftMax = 83; // maximum angle for turnin left
const int initialPos = 90; //not changable initial position of the servo motor

  int current_lane = 1;


void servoInit(){

  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myservo.write(initialPos); //sets 90 dgerees as a default value for starting position

}

void ServoTurnRight(){
///should the servo move gradually? 
  for (pos = initialPosition; pos >= rightMax; pos--) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  
    //should we wait for coming back to the initial wheels' position?
    for (pos = rightMax; pos >= rightMax; pos++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
}


void ServoTurnLeft(){
  //int IR1value = 1; you may want to change IR1 to IR2
   for (pos = initialPos; pos >= leftMax; pos--]) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

    //should we wait for coming back to the initial wheels' position?
    for (pos = leftMax; pos >= initialPos; pos++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
 

  
}






///Turning based on ultrasonic sesnor's readings.
///Here it chooses the proper lane based on the current one.
void ServoTurn(){
  ///distance is set by the ultrasonic sensor
  ///turn right and left are declared in the servo section
  if (distance <= 32 && distance >= 28){  //32, 28 are chosen min distances to start turning
    if(current_lane == 2){
      ServoTurnRight();
      current_lane = 1;
    }
    if(current_lane == 1){ //you may want to change which lane is which
      ServoTurnLeft();
      current_lane = 2;
    }
  }
}

