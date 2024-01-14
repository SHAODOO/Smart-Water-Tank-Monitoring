/*!
 * @file  SEN0204.ino
 * @brief  This example is to get liquid level. (Liquid Level Sensor-XKC-Y25-T12V)
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  jackli(Jack.li@dfrobot.com)
 * @version  V1.0
 * @date  2016-1-30
 */

#include <WiFi.h>

//wifi info
const char* ssid = "cs-mtg-room";
const char* password = "bilik703";
 
//Used Pins
const int lowLevelLiquidSensorPin = 38;
const int medLevelLiquidSensorPin = 39;
const int highLevelLiquidSensorPin = 40;
//const int analogPhMeter == ;

//define and initialize variables
bool lowWaterLevel = 0;
bool medWaterLevel = 0;
bool highWaterLevel = 0;
int waterLevel = 0;

void setupWifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void setup()
{
  Serial.begin(9600);
  setupWifi();
  pinMode(lowLevelLiquidSensorPin, INPUT);
  pinMode(medLevelLiquidSensorPin, INPUT);
  pinMode(highLevelLiquidSensorPin, INPUT);
}

void loop()
{
  lowWaterLevel = digitalRead(lowLevelLiquidSensorPin);
  medWaterLevel = digitalRead(medLevelLiquidSensorPin);
  highWaterLevel = digitalRead(highLevelLiquidSensorPin);

  if (lowWaterLevel == 0 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 0; // No water
  else if (lowWaterLevel == 1 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 1; // Low water level
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 0) 
    waterLevel = 2; // Medium water level
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 1) 
    waterLevel = 3; // High water level
  
  Serial.print("Water level = "); 
  Serial.println(waterLevel);
  delay(500);
}