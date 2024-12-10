//delay function without pasuing






///Infrared sensor
#define SENSOR_PIN 23


///SERVO

#include <ESP32Servo.h>

static const int servoPin = 13;

Servo myservo;
int pos = 90;

///ultrasonic sensor

const int trigPin = 5;
const int echoPin = 18;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
float distanceCm;




///COLOR SENSOR
#define S2 0 /*Define S2 Pin Number of ESP32*/
#define S3 4 /*Define S3 Pin Number of ESP32*/
#define sensorOut 9 /*Define Sensor Output Pin Number of ESP32*/
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
int IRvalue = 1;
uint8_t data[7];
   

///BLUETOOTH

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
int receivedValue =0; //initialized the received value from the app
///needed ti send values to the app
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }

  
};

/// Receiving values from the app
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
        Serial.print("Received value: ");
        for (char c : value) {
            Serial.print((uint8_t)c, HEX); // Print each byte as a hex value
            Serial.print(" ");
           receivedValue +=c;
           Serial.print("established:");
           Serial.print(receivedValue);
        }
        Serial.println();
    }
  }
};



void setup() {
Serial.begin(115200);
  CS_init();
  US_init();
  pinMode(SENSOR_PIN, INPUT); //IR init
  servoInit();
  pinMode(BATTERY_PIN, INPUT); //battery

  BTinit();
  

}

void loop() {

  CS_run();
  US_run();
  IR_run();
  BLE();
  BTrun();
  BatteryRead();
  //servoTurnLeft();
  // Serial.print("turning left");
  int state = digitalRead(BATTERY_PIN); // Read digital value
  Serial.println("The battery:");
  Serial.println(state);   
  
 // servoTurnRight();
   //Serial.println("turning right");
  
}

void BTinit(){

    

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
  );

  // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
  pCharacteristic->addDescriptor(new BLE2902());
  // Adds also the Characteristic User Description - 0x2901 descriptor
  descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription("My own description for this characteristic.");
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  pCharacteristic->addDescriptor(descriptor_2901);

   pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

}

void BTrun(){

   if (deviceConnected) {
  pCharacteristic->setValue((uint8_t *)data, 7);
    pCharacteristic->notify();
    value++;
    delay(500);
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                   // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

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
  delay(200); /*wait a 200mS*/
  Green = getGreen();
  delay(200); /*wait a 200mS*/
  Blue = getBlue();
  delay(200); /*wait a 200mS*/
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

void IR_run() {
  // read the state of the the input pin:
  int state = digitalRead(SENSOR_PIN);

  
    Serial.println("The obstacle is present - low: ");
    Serial.println(state);
    IRvalue = state;

  delay(100);
}


void BLE(){

  //uint8_t data[6];  // 1 identifier byte + 5 sensor bytes

    data[0] = 0x01;       // Identifier for sensor data packet
    data[1] = distanceCm;  // Example: Sensor 1 value (0-255)
    data[2] = distanceCm;  // Example: Sensor 2 value (0-255)
    data[3] = Red;  // Example: Sensor 3 value (0-255)
    data[4] = Green;  // Example: Sensor 4 value (0-255)
    data[5] = Blue; 
    data[6] = Vsource;// sending values *10
    //data[6] = batteryValue*10; //no float value
    //data[7] = servoValue;
 

}

void servoInit(){

  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myservo.write(90); //sets 90 dgerees as a default value for starting position

}

void servoTurnRight(){

   for (pos = 90; pos >= 180; pos++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
    if(pos==180){
    //should we wait for coming back to the initial wheels' position?
    for (pos = 180; pos >= 90; pos--) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

}
delay(1500);
}

void servoTurnLeft(){

   for (pos = 90; pos >= 0; pos--) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  if(pos==0){
    //should we wait for coming back to the initial wheels' position?
    for (pos = 0; pos >= 90; pos++) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  delay(1500);

  }
}

void avoidObstacle(){

  if(distanceCm<30){ //any other condition? from color sensor?
  servoTurnLeft();
  }
  else if(distanceCm<30){
   servoTurnRight();

  }
  
}


///DC motor 

void DCmotorInit(){

}

void DCmotorRun(){ //any speed?

  if(receivedValue == 1){
    //1 in the app is start
    //motor goes


  }
  else if( receivedValue  == 2){
    //motor stops, 2 is in the app as stop button
  }


}



//16, 3

void BatteryRead(){

double Vread = analogRead( BATTERY_PIN);
 int Rsmall = 3; //k
 int Rbig = 16;//k
 int Rsum = Rsmall+Rbig;
 int Vsource = Vread* rsum / rsmall * 10
 Serial.println(Vread);
 
 if(Vsource==135){
  //turn off the motor
 }

}


