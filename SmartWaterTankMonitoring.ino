#include <WiFi.h>
#include "DFRobot_ESP_PH.h"
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include "DHT.h"

const char* ssid = "cs-mtg-room";
const char* password = "bilik703";
const char *mqttServer = "34.172.68.58"; 
const int mqttPort = 1883;
const char *mqttTopic = "SmartWaterTankMonitoring";

const int lowLevelLiquidSensorPin = 38;
const int medLevelLiquidSensorPin = 39;
const int highLevelLiquidSensorPin = 40;
const int analogPhMeterPin = A0;
const int dhtSensorPin = 42;
const int buttonPin = 14;
const int servoMotorPin = 47;
const int rainSensorPin = 4;

#define DHTTYPE DHT11
DHT dht(dhtSensorPin, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
DFRobot_ESP_PH ph;
Servo myServo;

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
bool serverEmergencyStop = false;
int waterLeakage = 0;

void setupWifi() {
  delay(10);
  
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
  pinMode(buttonPin, INPUT);
  pinMode(rainSensorPin, INPUT);
  myServo.attach(servoMotorPin);  
  client.setServer(mqttServer, mqttPort);
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client"))
    {
      Serial.println("Connected to MQTT server");
    }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
 }
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

  if(digitalRead(buttonPin) == HIGH || serverEmergencyStop == true)
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
      serverEmergencyStop = false;
  }

  if (lowWaterLevel == 0 && medWaterLevel == 0 && highWaterLevel == 0) {
    waterLevel = 0; 
    if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
    }
  }
  else if (lowWaterLevel == 1 && medWaterLevel == 0 && highWaterLevel == 0) {
    waterLevel = 1; 
    if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
      }
  }    
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 0) 
  {
      waterLevel = 2; 
      if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
      }
  }    
  else if (lowWaterLevel == 1 && medWaterLevel == 1 && highWaterLevel == 1) 
  {
    waterLevel = 3;
    myServo.write(180);
    waterFillingInProgress = false; 
  }  
  
  voltage = analogRead(analogPhMeterPin) / 4096.0 * 3300;
  waterPh = ph.readPH(voltage); 
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  waterLeakage = !digitalRead(rainSensorPin);

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
  Serial.print("Water Leakage:");
  Serial.println(waterLeakage);
  Serial.println();

  if (!client.connected()) {
      reconnect();
  }
  client.loop();
  delay(5000); 

  int lowWaterLevelInt = lowWaterLevel ? 1 : 0;
  int medWaterLevelInt = medWaterLevel ? 1 : 0;
  int highWaterLevelInt = highWaterLevel ? 1 : 0;
  int waterLeakageInt = waterLeakage ? 1 : 0;
  int waterFillingInProgressInt = waterFillingInProgress ? 1 : 0;
  int pauseWaterFillingInt = pauseWaterFilling ? 1 : 0;

  char temperaturePayload[10];
  char humidityPayload[10];
  char waterPhPayload[10]; 

  sprintf(temperaturePayload, "%.2f", temperature);
  sprintf(humidityPayload, "%.2f", humidity);
  sprintf(waterPhPayload, "%.2f", waterPh);

  client.publish(MQTT_TOPIC "/LowWaterLevel", String(lowWaterLevelInt).c_str());
  client.publish(MQTT_TOPIC "/MedWaterLevel", String(medWaterLevelInt).c_str());
  client.publish(MQTT_TOPIC "/HighWaterLevel", String(highWaterLevelInt).c_str());
  client.publish(MQTT_TOPIC "/WaterLeakage", String(waterLeakageInt).c_str());
  client.publish(MQTT_TOPIC "/FillingInProgress", String(waterFillingInProgressInt).c_str());
  client.publish(MQTT_TOPIC "/PauseWaterFilling", String(pauseWaterFillingInt).c_str());
  client.publish(MQTT_TOPIC "/Temperature", temperaturePayload);
  client.publish(MQTT_TOPIC "/Humidity", humidityPayload);
  client.publish(MQTT_TOPIC "/WaterPh", waterPhPayload);

}