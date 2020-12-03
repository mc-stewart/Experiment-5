/**
OCADu - Fall 2020 - Creation & Computation
Experiment 5
Simran Duggal, Greg Martin, and Mairead Stewart
submitted to Kate Hartman and Nick Puckett
December 4, 2020

Based on code from Creation & Computation 'SendReceiveTemps'
https://github.com/DigitalFuturesOCADU/CC2020/tree/main/Experiment5

and 'six-axis-comp-filter'
https://github.com/DigitalFuturesOCADU/CC2020/tree/main/Experiment3
**/

#include <WiFiNINA.h>
#define PubNub_BASE_CLIENT WiFiClient
#include <PubNub.h>
#include <ArduinoJson.h>
#include <SparkFunLSM6DS3.h>
#include "six_axis_comp_filter.h"
#include "Wire.h"
#include <Servo.h>

float pitch;
float level;

int pos = 0;

bool canDoAction = true;
bool allLit = false;

Servo myservo;

const char* myID = "Mairead"; 
const static char publishChannel[] = "Mairead"; 
const static char readChannel1[] = "Greg";
const static char readChannel2[] = "Sim";

int firstLights[] = {12, 11, 10};     
int secondLights[] = {9, 8, 7};
int thirdLights[] = {6, 5, 4};  

const static char* teamInOrder[] = {publishChannel, readChannel1, readChannel2};
int teamStatuses[] = {0, 0, 0};
int numTeamAvail = 0;
int teamSize = 3;

extern char ssid[] = "CCRI_120Madison";
extern char pass[] = "Campus2020";
int status = WL_IDLE_STATUS;

char pubkey[] = "pub-c-9e59df52-dcc5-420e-ac6d-9cef140407e9";
char subkey[] = "sub-c-5793385a-3033-11eb-9d95-7ab25c099cb1";

StaticJsonDocument<200> dataToSend; 
StaticJsonDocument<200> inMessage;

String JsonParamName1 = "publisher";
String JsonParamName2 = "pitch";
unsigned long lastCheck;

int publishRate = 3000; 
unsigned long lastPublish;
int lastStatus = 0;
int thisStatus;

LSM6DS3 myIMU(I2C_MODE, 0x6A);
CompSixAxis CompFilter(0.1, 2);

void setup() {
  
  Serial.begin(9600);
  connectToPubNub();
  myservo.attach(A1);
  
  for (int i = 0; i < sizeof(firstLights) /sizeof(firstLights[0]); i++) {
    pinMode(firstLights[i], OUTPUT);
  }
  for (int i = 0; i < sizeof(secondLights)/sizeof(secondLights[0]); i++) {
    pinMode(secondLights[i], OUTPUT);  
  }
  for (int i = 0; i < sizeof(thirdLights) /sizeof(thirdLights[0]); i++) {
    pinMode(thirdLights[i], OUTPUT);
  }
  
  myIMU.begin();
  
}

void loop() {
  
  if((millis() - lastCheck) >= publishRate) {
    calculatePitchAndRoll();
    sendReceiveMessages();
    updateStatus();

    lastCheck = millis();
  }
  
  lightLevels(numTeamAvail);
}


void calculatePitchAndRoll()
{

  float accelX, accelY, accelZ,
      gyroX, gyroY, gyroZ,
      xAngle, yAngle; 
      
  accelX = myIMU.readFloatAccelX();
  accelY = myIMU.readFloatAccelY();
  accelZ = myIMU.readFloatAccelZ();
  
  gyroX = myIMU.readFloatGyroX();
  gyroY = myIMU.readFloatGyroY();
  gyroZ = myIMU.readFloatGyroZ();
 
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ);
  CompFilter.CompGyroUpdate(gyroX, gyroY, gyroZ); 
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  CompFilter.CompAnglesGet(&xAngle, &yAngle);
  
  pitch = xAngle*RAD_TO_DEG;
  
  if (pitch > 150) {
    thisStatus = 1;
    
  } else {
    thisStatus = 0;
    if (allLit) {
      
      canDoAction = true;
    }
  }
  
  teamStatuses[0] = thisStatus;
}

void sendReceiveMessages()
{
      if (thisStatus != lastStatus) {
        sendMessage();
        lastStatus = thisStatus;
      }
      
      readMessage(readChannel1);
      readMessage(readChannel2);
}

void updateStatus() {

    numTeamAvail = 0;
  
  for (int i = 0; i < sizeof(teamStatuses) / sizeof(teamStatuses[0]); i++) {
    if (teamStatuses[i] == 1) {      
      numTeamAvail++;
    }
  }
 
  if (numTeamAvail == teamSize) {
    allLit = true;
    completeMyAction();
  }
}

void lightLevels(int numLev) {
  int level = 255;
  
  int firstLightLevel = 0;
  int secondLightLevel = 0;
  int thirdLightLevel = 0;

  if (numTeamAvail > 0) {
    firstLightLevel = level;
    if (numTeamAvail > 1) {
      secondLightLevel = level;
      if (numTeamAvail > 2) {
        thirdLightLevel = level;
      }  
    }  
  }

  if (teamStatuses[1] == 1 && teamStatuses[2] == 1) {
    
    secondLightLevel = level;
    firstLightLevel = level;
    
  } else if (teamStatuses[1] == 1 || teamStatuses[2] == 1) {
    firstLightLevel = level;
  }

  for (int h = 0; h < sizeof(firstLights) /sizeof(firstLights[0]); h++) {
    digitalWrite(firstLights[h], firstLightLevel);
  }

  for (int i = 0; i < sizeof(secondLights) /sizeof(secondLights[0]); i++) { 
    digitalWrite(secondLights[i], secondLightLevel); 
  }

  for (int i = 0; i < sizeof(thirdLights) /sizeof(thirdLights[0]); i++) {
    digitalWrite(thirdLights[i], thirdLightLevel); 
  }
}

void completeMyAction() {
  if (canDoAction && allLit) {
    canDoAction = false;
  } else {
    
    for (pos = 0; pos <= 180; pos += 1) {
    myservo.write(pos);              
    delay(15);                       
    }
    for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);              
    delay(15);                       
    }
    
  }
  
}

void sendMessage() 
{
  char msg[64]; 

  dataToSend["publisher"] = myID; 
  dataToSend["pitch"] = thisStatus; 
  
  serializeJson(dataToSend, msg);
  
  WiFiClient* client = PubNub.publish(publishChannel, msg); 
  {
    Serial.println("publishing error"); 
  }
}

void readMessage(const char channel[]) {
  String msg;
    auto inputClient = PubNub.history(channel,1);
    if (!inputClient) 
    {
        Serial.println("message error");
        delay(1000);
        return;
    }
    HistoryCracker getMessage(inputClient);
    while (!getMessage.finished()) 
    {
        getMessage.get(msg);
        
        if (msg.length() > 0) 
        {

           deserializeJson(inMessage, msg);
           
           const char* inMessagePublisher =  inMessage[JsonParamName1];      // other person's name
           int inMessageStatus =        int(inMessage[JsonParamName2]); // other person's status (0 or 1)
           for (int i = 0; i < teamSize; i++) {
              Serial.print(String(teamInOrder[i]));
              Serial.print("-----");
              //Serial.print(String(inMessagePublisher));
              if (String(teamInOrder[i]) == String(inMessagePublisher)) {
                Serial.println(" TRUE");
                if (allLit && teamStatuses[i] != inMessageStatus) {
                  
                  canDoAction = true;
                }
                teamStatuses[i] = inMessageStatus;
              } else {
                Serial.println(" FALSE");
              }
              
           }
        }
    }
    inputClient->stop();
 
}

void connectToPubNub()
{
  while ( status != WL_CONNECTED) 
  {
    Serial.print("Attempting to connect to the network, SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    Serial.print("*");

    delay(2000);
  }

  Serial.println();
  Serial.print("You're connected to ");
  Serial.println(ssid);
  
  PubNub.begin(pubkey, subkey);
  Serial.println("Connected to PubNub Server");

}
