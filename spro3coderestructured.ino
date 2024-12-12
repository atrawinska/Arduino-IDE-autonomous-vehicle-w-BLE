//delay function without pasuing
// Timer struct for managing delay state
struct Timer {
  unsigned long previousMillis;
  unsigned long interval;
};

// Reusable delay function
bool nonBlockingDelay(Timer &timer) {
  unsigned long currentMillis = millis();
  if (currentMillis - timer.previousMillis >= timer.interval) {
    timer.previousMillis = currentMillis; // Update last run time
    return true; // Interval has passed
  }
  return false; // Interval not yet passed
}

// Functions using nonBlockingDelay

///Accelorometer
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


#include <Wire.h>
///Accelorometer, initial values
#define ACmeterSCL_PIN 22
#define ACmetrerSDA_PIN 21
  //delcares an object
  Adafruit_MMA8451 mma = Adafruit_MMA8451();

 float xacceleration = 0;
 float zacceleration = 0;

  float velocity = 0; 


///DC MOTOR
// Motor A
int motor1Pin1 = 33; 
int motor1Pin2 = 25; 
int enable1Pin = 26; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0; //?
const int resolution = 8;
int dutyCycle = 170;



///Infrared sensor
#define IR1_SENSOR_PIN 15
#define IR2_SENSOR_PIN 14
///there are also two values that take both IR signals, they are delcared as IRXvalue, X - 1 or 2

///SERVO

///ultrasonic sensor

const int trigPin = 29;
const int echoPin = 30;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
float distanceCm;




///COLOR SENSOR
#define S2 26 /*Define S2 Pin Number of ESP32*/
#define S3 27 /*Define S3 Pin Number of ESP32*/
#define sensorOut 24 /*Define Sensor Output Pin Number of ESP32*/
//works :)
#define BATTERY_PIN 36
int Vsource = 0;
//battery
/*Define int variables*/
int Red = 0;
int Green = 0;
int Blue = 0;
//int Frequency = 0;

 int R_Min = 0;
 int R_Max = 255;

///Values for BT sending

char currColor[50];
int IR1value = 1;
int IR2value = 1;
uint8_t data[7];
   


void setup() {
Serial.begin(115200);
  CS_init();
  US_init();
  pinMode(IR1_SENSOR_PIN, INPUT); //IR1 init
  pinMode(IR2_SENSOR_PIN, INPUT); //IR2 init
  servoInit();
  DCmotorInit();
  pinMode(BATTERY_PIN, INPUT); //battery
  //AccMeterInit();//accelorometer 

  BTinit();
  

}

void loop() {

  CS_run(); 
  US_run();
  IR1_run();
  IR2_run();
  BatteryRead();
  AccMeterRun();
  BLE(); //sets data to send
  BTrun();
  DCmotorRun();
  DetectRed();
  ServoTurn();
  
}




///////COLOUR SENSOR

void CS_init() {
  pinMode(S2, OUTPUT); /*Define S2 Pin as a OUTPUT*/
  pinMode(S3, OUTPUT); /*Define S3 Pin as a OUTPUT*/
  pinMode(sensorOut, INPUT); /*Define Sensor Output Pin as a INPUT*/
  Serial.begin(115200); /*Set the baudrate to 115200*/
  Serial.print("This is TCS3200 Calibration Code");
}

void CS_run() {
  Red = getRed();
  delay(10); /*wait a 200mS*/
  Green = getGreen();
  delay(10); /*wait a 200mS*/
  Blue = getBlue();
  delay(10); /*wait a 200mS*/
  Serial.print("Red Freq = ");
  Serial.print(Red); /*Print Red Color Value on Serial Monitor*/
  Serial.print("   ");
  Serial.print("Green Freq = ");
  Serial.print(Green); /*Print Green Color Value on Serial Monitor*/
  Serial.print("   ");
  Serial.print("Blue Freq = ");
  Serial.println(Blue); /*Print Blue Color Value on Serial Monitor*/
  snprintf(currColor, sizeof(currColor), "%d,%d,%d", Red, Green, Blue);
  
}

int getRed() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  int frequency = pulseIn(sensorOut, LOW); /*Get the Red Color Frequency*/

  Red = map(frequency, R_Min,R_Max,255,0);
   if(Red < 0){
    Red= 0;
   }
   else if(Red > 255){
    Red = 255;  
   }
  return Red;
}

int getGreen() {
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  int frequency = pulseIn(sensorOut, LOW); /*Get the Green Color Frequency*/
   Green = map(frequency, R_Min,R_Max,255,0);
   if(Green < 0){
    Green = 0;
   }
   else if(Green > 255){
    Green = 255;  
   }
  return Green;
}

int getBlue() {
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  int frequency = pulseIn(sensorOut, LOW); /*Get the Blue Color Frequency*/
  Blue= map(frequency, R_Min,R_Max,255,0);
   if(Blue < 0){
    Blue = 0;
   }
   else if(Blue > 255){
    Blue = 255;  
   }
  return Blue;
}

///ULTRASONIC

void US_init() {
  //Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void US_run() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;
  if(distanceCm > 100){
    distanceCm = 101;
  }
 
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distanceCm);
 
  
  delay(1000);
}

///infrared sensor

void IR1_run() {
  // read the state of the the input pin:
  int state = digitalRead(IR1_SENSOR_PIN);

  
    Serial.println("The obstacle is present - low: ");
    Serial.println(state);
    IR1value = state;

  delay(100);
}

void IR2_run() {
  // read the state of the the input pin:
  int state = digitalRead(IR2_SENSOR_PIN);

  
    Serial.println("The obstacle is present - low: ");
    Serial.println(state);
    IR2value = state;

  delay(100);
}








///////////////////////////////////////DC motor 

void DCmotorInit(){

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LEDC PWM
  //ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);

  
}

void DCmotorForward(){
   // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);


}

///Starts and stops the motor with BT
void DCmotorRun(){ //any speed?

  if(receivedValue == 1){
    //1 in the app is start
    //motor goes
    DCmotorForward();


  }
  else if( receivedValue  == 2){
    //motor stops, 2 is in the app as stop button
    
    DCmotorStop();
  }
}

void DCmotorStop(){

   Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);


}





/////////accelorometer

void AccMeterInit(){


  //init, it uses I2C 
  mma.begin();
  if (! mma.begin()) {
    Serial.println("Accelorometer couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");

  mma.setRange(MMA8451_RANGE_2_G);


}

void AccMeterRun(){

  sensors_event_t event; 
  mma.getEvent(&event);
  xacceleration = event.acceleration.x;
 zacceleration = event.acceleration.z;
  Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.println(" cm");
    CalculateVelocity();

}

void CalculateVelocity(){

  float acceleration = sqrt(xacceleration*xacceleration + zacceleration*zacceleration);

  float velocity = velocity + acceleration/2-0.07;
  Serial.print("Velocity: \t"); 
  Serial.print(velocity); 
  Serial.print("m/s \t");
  Serial.println("\t");
}



////////////BATTERY

void BatteryRead(){

double Vread = analogRead( BATTERY_PIN);
Vread = 3.3*Vread/4095;
 int Rsmall = 3; //k
 int Rbig = 16;//k
 int Rsum = Rsmall+Rbig;
 Vsource = Vread* Rsum / Rsmall * 10;
 Serial.println(Vread);
 Serial.println(Vsource);
 
 if(Vsource==135){
  //turn off the motor
  DCmotorStop();
  Serial.println("Motor stopped because of low battery");
 }

}


/////////////////////////////LOGICS

void DetectRed(){
///change the values of red, I tried to do it wide enought for different lighting
///initially it was Green<100 and Blue<100
if(Red>150 && (Green<150) && (Blue<150)){
  Serial.print("Red detected");
  DCmotorStop();

}
else{
 Serial.print("Red was not detected.");
}

}


