//Analog pH Meter power and ground got issue need to plog out and plugin

#include <WiFi.h>
#include "DFRobot_ESP_PH.h"
#include <ESP32Servo.h>
#include "DHT.h"

DFRobot_ESP_PH ph;

//define servo motor
Servo myServo;

//wifi info
const char* ssid = "cs-mtg-room";
const char* password = "bilik703";
 
//Used Pins
const int lowLevelLiquidSensorPin = 38;
const int medLevelLiquidSensorPin = 39;
const int highLevelLiquidSensorPin = 40;
const int analogPhMeterPin = A0;
const int dhtSensorPin = 42;

//input sensor
#define DHTTYPE DHT11
DHT dht(dhtSensorPin, DHTTYPE);

//define and initialize variables
bool lowWaterLevel = 0;
bool medWaterLevel = 0;
bool highWaterLevel = 0;
int waterLevel = 0;
int temperature = 0;
float voltage = 0;
float waterPh = 0;
float humidity = 0;


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
  ph.begin();
  dht.begin();
  pinMode(lowLevelLiquidSensorPin, INPUT);
  pinMode(medLevelLiquidSensorPin, INPUT);
  pinMode(highLevelLiquidSensorPin, INPUT);
  myServo.attach(21);
  
}

void loop()
{
  lowWaterLevel = digitalRead(lowLevelLiquidSensorPin);
  medWaterLevel = digitalRead(medLevelLiquidSensorPin);
  highWaterLevel = digitalRead(highLevelLiquidSensorPin);

  //TO DO LIST - Broken Sensor Detecting
  if (lowWaterLevel == 0 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 0; // No water
  else if (lowWaterLevel == 1 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 1; // Low water level
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 0) 
  {
      waterLevel = 2; // Medium water level
      myServo.write(0);
  }
    
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 1) 
  {
    waterLevel = 3; // High water level
    myServo.write(180);
  }  
  
  voltage = analogRead(analogPhMeterPin) / 4096.0 * 3300; // read the voltage
  waterPh = ph.readPH(voltage); // convert voltage to pH without temperature compensation
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  Serial.print("pH:");
  Serial.println(waterPh);
  Serial.print("Water level = "); 
  Serial.println(waterLevel);
  Serial.print("Temperature in the tank:");
  Serial.println(temperature);
  Serial.print("Humidity in the tank: "); 
  Serial.println(humidity);
  delay(500);
}