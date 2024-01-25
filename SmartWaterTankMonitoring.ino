#include <WiFi.h>
#include "DFRobot_ESP_PH.h"
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include "DHT.h"

//wifi info
const char* ssid = "cs-mtg-room";
const char* password = "bilik703";
const char *MQTT_SERVER = "34.172.68.58"; // your VM instance public IP address
const int MQTT_PORT = 1883;
const char *MQTT_TOPIC = "iot"; // MQTT topic
 
//Used Pins
const int lowLevelLiquidSensorPin = 38;
const int medLevelLiquidSensorPin = 39;
const int highLevelLiquidSensorPin = 40;
const int analogPhMeterPin = A0;
const int dhtSensorPin = 42;
const int buttonPin = 14;
const int servoMotorPin = 47;

//input sensor
#define DHTTYPE DHT11
DHT dht(dhtSensorPin, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

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
bool serverEmergencyStop = false;

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
  pinMode(buttonPin, INPUT);
  myServo.attach(servoMotorPin);  
  client.setServer(MQTT_SERVER, MQTT_PORT);
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

  //TO DO LIST - Broken Sensor Detecting
  if (lowWaterLevel == 0 && medWaterLevel == 0 && highWaterLevel == 0) {
    waterLevel = 0; // No water
    if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
    }
  }
  else if (lowWaterLevel == 1 && medWaterLevel == 0 && highWaterLevel == 0) {
    waterLevel = 1; // Low water level
    if (!pauseWaterFilling){
        myServo.write(0);
        waterFillingInProgress = true;
      }
  }    
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

  void loop()
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
    delay(5000); // adjust the delay according to your requirements
    float temperature = dht.readTemperature();
    char payload[10];
    sprintf(payload, "%.2f", temperature);
    client.publish(MQTT_TOPIC, payload);
  }
  
}