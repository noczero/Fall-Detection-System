/*
Bismillahirahmanirahim
To do list :
1. ESP8266 Wifi Connection (done)
2. MqTT Connection (done)
3. MPU

Wiring to MPU
------
GY-521  Wemos
MPu6050
board   D1 Pro Mini  Description
=====   ===========   ==========
VCC     VCC(5V)       Positive Voltage
GND     G             Ground / Negative Voltage
SCL     D1 (GPIO-05)  I2C Clock
SDA     D2 (GPIO-04)  I2C data
XDA     not used
XCL     not used
AD0     not used
INT     D8 (GPIO-15)  Interrupt Pin
*/
// Uncomment this to output 
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Import I2C MPU and its library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpuSensor;

// LED Define
#define internalLED 2 
#define redLED 16 //GPIO 16 - D0

// Wireless Define
const char* wifiSSID = "APTRG-LAB";
const char* wifiPassword = "gsglantaidua";

// MQTT Define
const char* mqttServerIP = "192.168.1.2";
const int mqttPort = 1883;

// define MPU cotrol/status vars
bool dmpReady = false; 
uint8_t mpuIntStatus; 
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation / motion vars
Quaternion quater;    // [w, x, y, z]         quaternion container
VectorInt16 acc;      // [x, y, z]            accel sensor measurements
VectorInt16 accReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 accWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; /// [x, y, z]            gravity vector

#ifdef OUTPUT_READABLE_EULER
float euler[3]; // [psi, theta, phi]    Euler angle container
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
float yawPitchRoll[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif

// pin MPU
#define INTERRUPT_PIN 15 // use pin 15 on ESP8266
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// procedure MPU
void dmpDataReady(){
  mpuInterrupt = true;
}

void mpuSetup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // Initialize Devie 
  Serial.println("Initializing i2c devices MPU");
  mpuSensor.initialize();
  pinMode(INTERRUPT_PIN, INPUT); // int become input

  // verify connection
  Serial.println("Testing device Connections...");
  Serial.println(mpuSensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // load and configure the DMP
  Serial.println("Initializing DMP....");
  devStatus = mpuSensor.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
  mpuSensor.setXGyroOffset(220);
  mpuSensor.setYGyroOffset(76);
  mpuSensor.setZGyroOffset(-85);
  mpuSensor.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpuSensor.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpuSensor.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpuSensor.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

}
// delay milis
int valueRandom = 0;
int lastMsg = 0;
// untuk pesan message
String temp_str;
String hum_str;
char temp[50];
char hum[50];

WiFiClient myESP;
PubSubClient client(myESP);

void wifiSetup(){
  WiFi.begin(wifiSSID,wifiPassword);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("Waiting, connection to Wifi..");
    Serial.print("SSID : "); Serial.println(wifiSSID);
    
    // give notification LED
    redLEDHL(); 
  }
  Serial.println("Connected to the WiFI Network "); 
  Serial.print("Connected Network "); Serial.println(wifiSSID);
  Serial.print("IP Local "); Serial.println(WiFi.localIP());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  wifiSetup();

  delay(100);
  mpuSetup();

  //Initialize MQTT Connection
  client.setServer(mqttServerIP, mqttPort);
  client.setCallback(callback); // callback for incoming message
  pinMode(internalLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

// main MPU
void mainMPU(){
  // if programming setup MPU failed, just return
  if(!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if(!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpuSensor.getIntStatus();

  // get current FIFO Count
  fifoCount = mpuSensor.getFIFOCount();

  // check for overflow
  if((mpuIntStatus & 0x10) || fifoCount == 1024){
    // reset 
    mpuSensor.resetFIFO();
    Serial.println("FIFO Overflod!");

    // otherwise, check for DMP data ready interrupt
  } else if(mpuIntStatus & 0x02){

    // wait for correct available data, length
    while(fifoCount < packetSize)
      fifoCount = mpuSensor.getFIFOCount();

    // read a packet from FIFO
    mpuSensor.getFIFOBytes(fifoBuffer, packetSize);

    //track FIFO Count in case there is > 1 packe available
    fifoCount -+ packetSize;

    // print Value
    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpuSensor.dmpGetQuaternion(&quater, fifoBuffer);
      Serial.print("quaterion \t");
      Serial.print(quater.w);
      Serial.print("\t");
      Serial.print(quater.x);
      Serial.print("\t");
      Serial.print(quater.y);
      Serial.print("\t");
      Serial.println(quater.z);
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpuSensor.dmpGetQuaternion(&quater, fifoBuffer);
      mpuSensor.dmpGetGravity(&gravity, &quater);
      mpuSensor.dmpGetYawPitchRoll(yawPitchRoll, &quater, &gravity);
      Serial.print("Yaw Pitch Roll\t");
      Serial.print(yawPitchRoll[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(yawPitchRoll[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(yawPitchRoll[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpuSensor.dmpGetQuaternion(&quater, fifoBuffer);
      mpuSensor.dmpGetAccel(&acc, fifoBuffer);
      mpuSensor.dmpGetGravity(&gravity, &quater);
      mpuSensor.dmpGetLinearAccel(&accReal, &acc, &gravity);
      Serial.print("Accelero Real\t");
      Serial.print(accReal.x);
      Serial.print("\t");
      Serial.print(accReal.y);
      Serial.print("\t");
      Serial.println(accReal.z);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpuSensor.dmpGetQuaternion(&quarter, fifoBuffer);
      mpuSensor.dmpGetAccel(&acc, fifoBuffer);
      mpuSensor.dmpGetGravity(&gravity, &quarter);
      mpuSensor.dmpGetLinearAccel(&accReal, &acc, &gravity);
      mpuSensor.dmpGetLinearAccelInWorld(&accWorld, &accReal, &quarter);
      Serial.print("Accelerometer World\t");
      Serial.print(accWorld.x);
      Serial.print("\t");
      Serial.print(accWorld.y);
      Serial.print("\t");
      Serial.println(accWorld.z);
    #endif
  }

}


void reconnect(){
  // MQTT Begin
  while(!client.connected()){
    Serial.println("Connecting to MQTT Server..");
    Serial.print("IP MQTT Server : "); Serial.println(mqttServerIP);

    bool hasConnection = client.connect("ZeroESP-1"); // connect(id,username,password) -> true if connect
    if(hasConnection){
      Serial.println("Success connected to MQTT Broker");
    } else {
      Serial.print("Failed connected");
      Serial.println(client.state());
      delay(2000);
      Serial.println("Try to connect...");
    }
  }
  client.publish("esp/test", "Reconnecting");
  client.subscribe("esp/test");
}

void callback(char* topic, byte* payload, unsigned int length){
  Serial.println("--------");
  Serial.println("Message Arrived");
  Serial.print("Topic :"); Serial.println(topic);
  Serial.print("Message : ");
  for(int i=0; i < length; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("--------");
}

void notifLED(){
  digitalWrite(internalLED,HIGH);
  delay(300);
  digitalWrite(internalLED,LOW);
}

void redLEDHL(){
  digitalWrite(redLED, HIGH);
  delay(100);
  digitalWrite(redLED, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (!client.connected()){
    reconnect();
  }
  valueRandom = random(10,100);
  client.loop(); //looping forever the client
  
  // publish every 2 secondss
  long now = millis(); 
  if (now-lastMsg > 2000){
    lastMsg = now;
  
    // value convert to string
    temp_str = String(valueRandom); 
    temp_str.toCharArray(temp, temp_str.length() + 1);
    
    client.publish("esp/test", temp); //topic esp/test message : temp (string)
    String message = "Publish message to server : " + String(valueRandom);
    Serial.println(message);
    notifLED();
  }

  // call MPU;
  mainMPU();
      redLEDHL(); 

  delay(50);
}
