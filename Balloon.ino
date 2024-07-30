#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoJson.h>

int ledPin = 11;
int ledValue = 0;
int receivedValue;

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "meshfestival.cloud.shiftr.io";
int port = 1883;
const char topicGyro[] = "/gyro";
const char topicLED[] = "/led";
const char topicInterval[] = "/interval";

int long interval = 1000;
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  //define LED pin
  pinMode(ledPin, OUTPUT);
  analogWrite(ledPin, ledValue);

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  // Each client must have a unique client ID
  mqttClient.setId("arduino");

  // You can provide a username and password for authentication
  mqttClient.setUsernamePassword("meshfestival", "ddK8j8sfFMRL0SQy");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1)
      ;
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.print("Subscribing to topic: ");
  Serial.println(topicLED);
  Serial.print("Subscribing to topic: ");
  Serial.println(topicInterval);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(topicLED);
  mqttClient.subscribe(topicInterval);
  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();
  receiveValues();
  readGyroValues();
}

void receiveValues() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    String topic = mqttClient.messageTopic();
    Serial.print("Received message on topic: ");
    Serial.print(topic);
    Serial.print(", length: ");
    Serial.print(messageSize);
    Serial.println(" bytes");

    String payload = "";
    while (mqttClient.available()) {
      payload += (char)mqttClient.read();
    }

    if (topic.equals("/led")) {
      int valueLed = payload.toInt();
      analogWrite(ledPin,  valueLed);
      Serial.println(valueLed);
    
    } else if (topic.equals("/interval")) {
      int valueInterval = payload.toInt();
      interval= valueInterval;
      Serial.println(valueInterval);
    }

  }
}

void readGyroValues() {
  
  unsigned long currentMillis = millis();
  float gyroX, gyroY, gyroZ;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        // Serial.print("Gyroscope values: ");
        // Serial.print("x: "); Serial.print(gyroX);
        // Serial.print("y: "); Serial.print(gyroY);
        // Serial.print("z: "); Serial.println(gyroZ);

        // Create a JSON document
        StaticJsonDocument<200> doc;
        doc["x"] = gyroX;
        doc["y"] = gyroY;
        doc["z"] = gyroZ;

        // Serialize JSON document to a string
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);
   
        mqttClient.beginMessage(topicGyro);
        mqttClient.print(jsonBuffer);
        mqttClient.endMessage();
    }
  }
}
