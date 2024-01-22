//Analog pH Meter power and ground got issue need to plog out and plugin

#include <WiFi.h>
#include "DFRobot_ESP_PH.h"
#include <ESP32Servo.h>
#include "DHT.h"
#include "VOneMqttClient.h"

//wifi info
const char* ssid = "cs-mtg-room";
const char* password = "bilik703";
 
//Used Pins
const int lowLevelLiquidSensorPin = 38;
const int medLevelLiquidSensorPin = 39;
const int highLevelLiquidSensorPin = 40;
const int analogPhMeterPin = A0;
const int dhtSensorPin = 42;
const int buttonPin = 14;
const int servoMotorPin = 21;

//input sensor
#define DHTTYPE DHT11
DHT dht(dhtSensorPin, DHTTYPE);

DFRobot_ESP_PH ph;

//define servo motor
Servo myServo;

//define and initialize variables
bool lowWaterLevel = 0;
bool medWaterLevel = 0;
bool highWaterLevel = 0;
int waterLevel = 0;
int temperature = 0;
float voltage = 0;
float waterPh = 0;
float humidity = 0;
bool waterFillingInProgress = false;
bool pauseWaterFilling = false;
bool sameState = false;

//define device id
const char* dhtSensorId = "7f4427ef-afa4-4d08-9d7a-bec2d13f5368";  
const char* lowLevelLiquidSensorId = "9cb5226e-2d17-4aca-9f14-bbeaed0adf34";
const char* medLevelLiquidSensorId = "2bae4568-52e2-4c89-ab0c-9ba5928f81ca";
const char* highLevelLiquidSensorId = "a529d171-7b76-4011-bebb-f55015db35ef";
const char* analogPhMeterId = "351270bd-1f2f-4875-90c9-460a8c7bc672";
const char* servoMotorId = "b6c64ef0-5bc7-4682-bb8e-a70dc6543810";

//Create an instance of VOneMqttClient
VOneMqttClient voneClient;

//last message time
unsigned long lastMsgTime = 0;

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

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand)
{
  //actuatorCommand format {"servo":90}
  Serial.print("Main received callback : ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";

  JSONVar commandObjct = JSON.parse(actuatorCommand);
  JSONVar keys = commandObjct.keys();

  if (String(actuatorDeviceId) == servoMotorId)
  {
    //{"servo":90}
    String key = "";
    JSONVar commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char* )keys[i];
      commandValue = commandObjct[keys[i]];

    }
    Serial.print("Key : ");
    Serial.println(key.c_str());
    Serial.print("value : ");
    Serial.println(commandValue);

    int angle = (int)commandValue;
    myServo.write(angle);
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);//publish actuator status
  }
  else
  {
    Serial.print(" No actuator found : ");
    Serial.println(actuatorDeviceId);
    errorMsg = "No actuator found";
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);//publish actuator status
  }
}

void setup()
{
  Serial.begin(9600);
  setupWifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  ph.begin();
  dht.begin();
  pinMode(lowLevelLiquidSensorPin, INPUT);
  pinMode(medLevelLiquidSensorPin, INPUT);
  pinMode(highLevelLiquidSensorPin, INPUT);
  pinMode(buttonPin, INPUT);
  myServo.attach(servoMotorPin);  
}

void loop()
{    
  lowWaterLevel = digitalRead(lowLevelLiquidSensorPin);
  medWaterLevel = digitalRead(medLevelLiquidSensorPin);
  highWaterLevel = digitalRead(highLevelLiquidSensorPin);

  if (digitalRead(buttonPin) == LOW)
  {
    sameState = false;
  }

  if(digitalRead(buttonPin) == HIGH)
  {
    if (sameState == false)
    {
      if (pauseWaterFilling == true)
      {
        pauseWaterFilling = false;
        //put light off
      }

      if(waterFillingInProgress == true)
      {
        myServo.write(180);
        waterFillingInProgress = false;
        pauseWaterFilling = true;
        //put light on
      }
    }
      sameState = true;
  }

  //TO DO LIST - Broken Sensor Detecting
  if (lowWaterLevel == 0 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 0; // No water
  else if (lowWaterLevel == 1 && medWaterLevel == 0 && highWaterLevel == 0) 
    waterLevel = 1; // Low water level
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 0) 
  {
      waterLevel = 2; // Medium water level
      if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
      }
  }    
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 1) 
  {
    waterLevel = 3; // High water level
    myServo.write(180);
    waterFillingInProgress = false; 
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
  Serial.print("pause:");
  Serial.println(pauseWaterFilling);
  Serial.print("filling:");
  Serial.println(waterFillingInProgress);
  Serial.println();

  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(dhtSensorId, true);
    voneClient.publishDeviceStatusEvent(lowLevelLiquidSensorId, true);
    voneClient.publishDeviceStatusEvent(medLevelLiquidSensorId, true);
    voneClient.publishDeviceStatusEvent(highLevelLiquidSensorId, true);
    voneClient.publishDeviceStatusEvent(analogPhMeterId, true);
  }
  voneClient.loop();

  unsigned long cur = millis();
  if (cur - lastMsgTime > INTERVAL) {
    lastMsgTime = cur;

    JSONVar payloadObject;
    payloadObject["Humidity"] = humidity;
    payloadObject["Temperature"] = temperature;
    voneClient.publishTelemetryData(dhtSensorId, payloadObject);
    voneClient.publishTelemetryData(lowLevelLiquidSensorId, "Liquid level", lowWaterLevel);
    voneClient.publishTelemetryData(medLevelLiquidSensorId, "Liquid level", medWaterLevel);
    voneClient.publishTelemetryData(highLevelLiquidSensorId, "Liquid level", highWaterLevel);
    voneClient.publishTelemetryData(analogPhMeterId, "pH", waterPh);
  }
}