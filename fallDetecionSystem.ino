#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define internalLED 2

const char* wifiSSID = "APTRG-LAB";
const char* wifiPassword = "gsglantaidua";

const char* mqttServerIP = "192.168.1.2";
const int mqttPort = 1883;


int valueRandom = 0;
int lastMsg = 0;
// untuk pesan message
String temp_str;
String hum_str;
char temp[50];
char hum[50];

//MPU
#include <Wire.h>
const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;

WiFiClient myESP;
PubSubClient client(myESP);

void MPUSetup(){
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void wifiSetup(){
  WiFi.begin(wifiSSID,wifiPassword);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("Waiting, connection to Wifi..");
    Serial.print("SSID : "); Serial.println(wifiSSID);
  }
  Serial.println("Connected to the WiFI Network"); 
  Serial.print("Connected Network "); Serial.println(wifiSSID);
  Serial.print("IP Local "); Serial.println(WiFi.localIP());
}

void MPUMain(){
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  Serial.print("AccX = "); Serial.print(AccX);
  Serial.print(" || AccY = "); Serial.print(AccY);
  Serial.print(" || AccZ = "); Serial.print(AccZ);
  Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);
  Serial.print(" || GyroX = "); Serial.print(GyroX);
  Serial.print(" || GyroY = "); Serial.print(GyroY);
  Serial.print(" || GyroZ = "); Serial.println(GyroZ);
  delay(100);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  wifiSetup();
  MPUSetup();
  //Initialize MQTT Connection
  client.setServer(mqttServerIP, mqttPort);
  client.setCallback(callback); // callback for incoming message
  pinMode(internalLED, OUTPUT);
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
  MPUMain();
}


