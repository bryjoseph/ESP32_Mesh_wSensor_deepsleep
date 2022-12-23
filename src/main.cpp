#include <ESP32Servo.h>
// Include Wire Library for I2C
#include <Wire.h>
// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_TMP117.h> // Used to send and recieve specific information from TMP117 sensor

/**Define the Wi-Fi and MQTT BLOCK**/
// Enter WiFi SSID and password
#define WIFI_SSID "RT-AC1200_48_2G"
#define WIFI_PASSWORD "RoxSun10122021"

// Raspberry Pi Mosquitto (MQTT) Broker
#define MQTT_HOST IPAddress(192, 168, 50, 249)
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_FTEMP "esp/sparkfuntmp117/fahrenheittemperature"
#define MQTT_PUB_CTEMP "esp/sparkfuntmp117/celsiustemperature"
#define MQTT_SUB_ZONE1_COMMAND "esp/zone/1/command"

// Variables to hold sensor readings
float temperatureF;
float temperatureC;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

// The default address of the device is 0x48 = (GND)
TMP117 sensor; // Initalize sensor using I2C

/**Define the PCA9685 and SERVO BLOCK**/
// Creat object to represent PCA9685 at default I2C address 0x40
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match servos
#define SERVOMIN  90 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  8  //Servo Motor 1 on connector 8

// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;

/**INIT METHODS BLOCK**/
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
  Serial.println("Connecting to strongest Wi-Fi network...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT Broker...");
  mqttClient.connect();
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void executeServoOpenCommand() {
  // Move Motor 1 from 90 to 0 degrees -- Motor 1 is currently holding the vent open
  for (int posDegrees = 90; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    // Print to serial monitor
    // Serial.print("Motor 1 = ");
    // Serial.println(posDegrees);
    // delay(30);
  }

  // Move Motor 0 from 0 to 90 degrees -- Motor 0 will close vent after Motor 1 moves out of the way
  for (int posDegrees = 0; posDegrees <= 90; posDegrees++) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    // Print to serial monitor
    // Serial.print("Motor 0 = ");
    // Serial.println(posDegrees);
    // delay(30);
  }
}

void executeServoCloseCommand() {
  // Move Motor 0 from 90 to 0 degrees
  for (int posDegrees = 90; posDegrees >= 0; posDegrees--) {
 
    // Determine PWM pulse width
    pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER0, 0, pwm0);
    // Print to serial monitor
    // Serial.print("Motor 0 = ");
    // Serial.println(posDegrees);
    // delay(30);
  }
 
  // Move Motor 1 from 0 to 90 degrees
  for (int posDegrees = 0; posDegrees <= 90; posDegrees++) {
 
    // Determine PWM pulse width
    pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
    // Write to PCA9685
    pca9685.setPWM(SER1, 0, pwm1);
    // Print to serial monitor
    // Serial.print("Motor 1 = ");
    // Serial.println(posDegrees);
    // delay(30);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Message received from subscribed topic");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
}

void mqttMessageCallback(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String recMessage;
  
  for (int i = 0; i < len; i++) {
    Serial.print(payload[i]);
    recMessage += payload[i];
  }
  Serial.println();

  // A message received on the topic esp/zone/1/command, check if the message is either "on" or "off" 
  // Causes servo movement according to the message
  Serial.print("Changing output to ");
  if (recMessage == "open") {
    Serial.println("open");
    executeServoOpenCommand();
  }
  else if (recMessage == "close") {
    Serial.println("close");
    executeServoCloseCommand();
  }
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

  // in the MQTT connect call the subscribe as well
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_SUB_ZONE1_COMMAND, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
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
/**END METHODS BLOCK**/

void setup() {
  //Initialize serial and wait for port to open:
  Wire1.begin();
  // Serial monitor setup
  Serial.begin(115200);
  // Initialize PCA9685
  pca9685.begin();
  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(SERVO_FREQ);

  Wire1.setClock(400000);   // Set clock speed to be the fastest for better communication (fast mode)
  Serial.println();

  // Print to monitor
  Serial.println("TMP117: Basic Readings with PCA9685 Servo Test");
  
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
  mqttClient.onSubscribe(onMqttSubscribe);
  // mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(mqttMessageCallback);
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
    Serial.printf("Date and Time of Message: %.2f \n", temperatureC);

    // Publish an MQTT message on topic esp/sparkfuntmp117/fahrenheittemperature
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_FTEMP, 1, true, String(temperatureF).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_FTEMP, packetIdPub2);
    Serial.printf("Date and Time of Message: %.2f \n", temperatureF);
  }
}