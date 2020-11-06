/*
   Basic BME280 MQTT client -> publish sensor data to an MQTT server
*/

// BME280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

// WiFi
#include <ESP8266WiFi.h>
#include "wifi_secrets.h"

const char* wifi_ssid = WIFI_SSID;
const char* wifi_pass = WIFI_PASS;

// MQTT
#include <ArduinoMqttClient.h>
#include "mqtt_config.h"

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqtt_broker[] = MQTT_BROKER;
const char mqtt_topic[] = MQTT_TOPIC;


void setup() {
  // setup serial port
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  // setup sensor
  Serial.println("BME280 MQTT Client startup.");

  if (!bme.begin(0x76, &Wire)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    // fatal
    while (1)
      delay(1e6);
  }

  Serial.println("BME280 Sensor ready");

  // wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    // retry
    delay(1000);
  }

  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

#if DEEPSLEEP == 1
  measure_and_send();
  // allow mqtt to deliver message
  delay(5);
  ESP.deepSleep(SLEEP_INTERVAL * 1e6);
#endif
}

void measure_and_send() {
    sensors_event_t temp_event, pressure_event, humidity_event;

#if DEEPSLEEP != 1
  // If we're deep sleeping, we will just have setup the wifi, otherwise
  // force a wifi reconnect since we might be disconnected
  if (WiFi.status() != WL_CONNECTED)
    WiFi.reconnect();
#endif

  // fetch sensor data
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  String msg = "{\"t\":\"" + String(temp_event.temperature) +
               "\",\"h\":\"" + String(humidity_event.relative_humidity) +
               "\",\"p\":\"" + String(pressure_event.pressure) +
               "\"}";

  Serial.println(msg);

  // send MQTT data
  mqttClient.connect(mqtt_broker, MQTT_PORT);
  mqttClient.beginMessage(mqtt_topic);
  mqttClient.print(msg);
  mqttClient.endMessage();
  // for some reason the mqttClient library has .disconnect() as private
}


void loop() {
#if DEEPSLEEP != 1
  // go to sleep to preserve power
  delay(SLEEP_INTERVAL * 1e3);
#endif
}
