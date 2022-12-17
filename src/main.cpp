
#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from our sensor

// Enter WiFi SSID and password
#define WIFI_SSID "RT-AC1200_48_2G"
#define WIFI_PASSWORD "RoxSun10122021"
// char ssid[] = "RT-AC1200_48_2G";             // your network SSID (name)
// char pass[] = "RoxSun10122021";    // your network password (use for WPA, or use as key for WEP)
// int keyIndex = 0;                      // your network key Index number (needed only for WEP)

// Raspberry Pi Mosquitto (MQTT) Broker
#define MQTT_HOST IPAddress(192, 168, 50, 249)
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_FTEMP "esp/sparkfuntmp117/fahrenheittemperature"
#define MQTT_PUB_CTEMP "esp/sparkfuntmp117/celsiustemperature"

// The default address of the device is 0x48 = (GND)
TMP117 sensor; // Initalize sensor using I2C

// Variables to hold sensor readings
float temperatureF;
float temperatureC;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

// get sensor readings here
void getSparkfunTMP117Readings() {
  // Tell TMP117 to begin measurements
  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    temperatureC = sensor.readTempC();
    temperatureF = sensor.readTempF();
  }
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT Broker...");
  mqttClient.connect();
}

// this switch is going to call a secondary action if a WiFi event occurs
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT Broker.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Acknowledging Message Publish.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  //Initialize serial and wait for port to open:
  Wire.begin();
  Serial.begin(115200);
  Wire.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  Serial.println();

  Serial.println("TMP117: Basic Readings");
  if (sensor.begin() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println("Begin");
  }
  else
  {
    Serial.println("Device failed to setup- Freezing code.");
    while (1); // Runs forever
  }

  // timers that allow both the MQTT broker and Wi-Fi connection to reconnect if connection is lost
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // capture WiFi event and send to switch
  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    getSparkfunTMP117Readings();
    Serial.println();
    // print for reference/debug
    Serial.printf("Temperature = %.2f ºC \n", temperatureC);
    Serial.printf("Temperature = %.2f ºF \n", temperatureF);

    // Publish an MQTT message on topic esp/sparkfuntmp117/celsiustemperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_CTEMP, 1, true, String(temperatureC).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_CTEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperatureC);

    // Publish an MQTT message on topic esp/sparkfuntmp117/fahrenheittemperature
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_FTEMP, 1, true, String(temperatureF).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_FTEMP, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperatureF);
  }
}