#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Secrets.h>

#define DHTTYPE DHT11
#define LEDPIN 2
#define DHTPIN 14
#define ONEWIREPIN 4

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
WiFiClient wifiClient;
PubSubClient client(mqtt_server, mqtt_port, wifiClient);
OneWire oneWire(ONEWIREPIN);
DallasTemperature bucketSensors(&oneWire);
NTPClient timeClient(udp, "north-america.pool.ntp.org", utc_offset);

char daysOfWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char* wifi_ssid = PSK_WIFI_SSID;
const char* wifi_password = PSK_WIFI_PASSWD;
const char* mqtt_server = "octopi.local";
const char* mqtt_user = PSK_MQTT_USER;
const char* mqtt_password = PSK_MQTT_PASSWD;
const char* clientID = "hydro";
const char* topicTempF = "/hydro/tempF";
const char* topicTempC = "/hydro/tempC";
const char* topicHumidity = "/hydro/humidity";
const char* topicBucket1 = "/hydro/bucket1";
const char* topicBucket2 = "/hydro/bucket2";
const char* topicUptime = "/hydro/uptime";
const int mqtt_port = 1883;
const long utc_offset = -25000; //offset to UTC -7:00
const unsigned long interval = 60000;
unsigned long uptime;
unsigned long startTimer;
float bucketTemp1;
float bucketTemp2;
float humidity;
float tempC;
float tempF;

void getTime() {
  timeClient.update();

  Serial.println("---- " + (String)daysOfWeek[timeClient.getDay()] + ", " + timeClient.getFormattedTime() + " ----");
  Serial.print("Uptime: ");
  Serial.println(uptime);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(wifi_ssid);
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  Serial.print("Connecting to MQTT broker");

  while(!client.connected()) {
    delay(500);
    Serial.print(".");

    if(client.connect(clientID, mqtt_user, mqtt_password)) {
      Serial.println();
      Serial.print("Connected to MQTT as client: ");
      Serial.println(clientID);
      client.subscribe(topicHumidity);
      client.subscribe(topicTempF);
      client.subscribe(topicBucket1);
      client.subscribe(topicBucket2);
      client.subscribe(topicUptime);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("[");
  Serial.print(topic);
  Serial.print("] ");

  for(unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(wifi_ssid, wifi_password);
  timeClient.begin();
  bucketSensors.begin();
  dht.begin();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  uptime = millis();

  while(WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  while(!client.connected()) {
    connectMQTT();
  }

  if(uptime - startTimer >= interval) {
    getTime();
    
    humidity = dht.readHumidity();
    tempC = dht.readTemperature();
    tempF = dht.readTemperature(true);

    if(isnan(humidity) || isnan(tempC) || isnan(tempF)) {
      Serial.println("Failed to read from DHT sensor.");
      return;
    }

    bucketSensors.requestTemperatures();
    bucketTemp1 = bucketSensors.getTempFByIndex(0);
    bucketTemp2 = bucketSensors.getTempFByIndex(1);

    if(bucketTemp1 < 0)
      Serial.println("Failed to read Bucket 1 temperature.");
    else if(bucketTemp2 < 0) 
      Serial.println("Failed to read Bucket 2 temperature.");

    client.publish(topicHumidity, String(humidity).c_str());
    client.publish(topicTempF, String(tempF).c_str());
    client.publish(topicBucket1, String(bucketTemp1).c_str());
    client.publish(topicBucket2, String(bucketTemp2).c_str());
    client.publish(topicUptime, String(uptime).c_str());

    Serial.print("DHT11 Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    Serial.print("DHT11 Temp (F): ");
    Serial.println(tempF);
    Serial.print("Bucket 1 Temp (F): ");
    Serial.println(bucketTemp1);
    Serial.print("Bucket 2 Temp (F): ");
    Serial.println(bucketTemp2);

    startTimer = uptime;
  }
  
  client.loop();
}