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
const char mqtt_prefix[] = MQTT_PREFIX;

String mqtt_status;
String mqtt_config;

String mac;
String devID;

int configAnnounce = 0;

void setup() {
  // setup serial port
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // setup sensor
  Serial.println("BME280 MQTT Client startup.");

  // bme280 module directly soldered on nodemcu, at pins d6, d5, gnd, 3v3!
  Wire.begin(D6, D5);

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

  mac = WiFi.macAddress();
  devID = String(mqtt_prefix) + "-" + mac.substring(0,2) + mac.substring(3,5) + mac.substring(6,8)
            +mac.substring(9,11) + mac.substring(12,14) + mac.substring(15,17);

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
  String msg;
  String topic;
  sensors_event_t temp_event, pressure_event, humidity_event;

#if DEEPSLEEP != 1
  // If we're deep sleeping, we will just have setup the wifi, otherwise
  // force a wifi reconnect since we might be disconnected
  if (WiFi.status() != WL_CONNECTED)
    WiFi.reconnect();
#endif

  // setup status topic for us in discovery data
  mqtt_status = "sensor/" + String(devID) + "/status";
  // send MQTT autodiscover data

  // hop online and send
  mqttClient.connect(mqtt_broker, MQTT_PORT);

  if (configAnnounce == 0) {
    Serial.println("Announcing autodiscovery configs...");

    // temp config
    topic = String(DISCOVERY_PREFIX) + "/sensor/" + devID + "/temperature" + "/config";
    msg = String("{\"device_class\": \"temperature\", \"name\": \""
                        + devID +
                        " - Temperature\", \"unique_id\": \""
                        + devID + 
                        "-temperature\", \"state_topic\": \""
                        + mqtt_status +
                        "\", \"unit_of_measurement\": \"°C\", \"value_template\": \"{{ value_json.t}}\" }");

    mqttClient.beginMessage(topic, (unsigned long)msg.length());
    mqttClient.print(msg);
    mqttClient.endMessage();

    // humidity config
    topic = String(DISCOVERY_PREFIX) + "/sensor/" + devID + "/humidity" + "/config";
    msg = String("{\"device_class\": \"humidity\", \"name\": \""
                        + devID +
                        " - Humidity\", \"unique_id\": \""
                        + devID + 
                        "-humidity\", \"state_topic\": \""
                        + mqtt_status +
                        "\", \"unit_of_measurement\": \"%\", \"value_template\": \"{{ value_json.h}}\" }");

    mqttClient.beginMessage(topic, (unsigned long)msg.length());
    mqttClient.print(msg);
    mqttClient.endMessage();

    // pressure config
    topic = String(DISCOVERY_PREFIX) + "/sensor/" + devID + "/pressure" + "/config";
    msg = String("{\"device_class\": \"pressure\", \"name\": \""
                        + devID +
                        " - Pressure\", \"unique_id\": \""
                        + devID + 
                        "-pressure\", \"state_topic\": \""
                        + mqtt_status +
                        "\", \"unit_of_measurement\": \"hPa\", \"value_template\": \"{{ value_json.p}}\" }");

    mqttClient.beginMessage(topic, (unsigned long)msg.length());
    mqttClient.print(msg);
    mqttClient.endMessage();
  }

  // only announce autodiscovery every CONFIG_ANNOUNCE_INTERVAL
  configAnnounce = (configAnnounce + 1) % CONFIG_ANNOUNCE_INTERVAL;

  // fetch sensor data
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  // actual status
  msg = "{\"t\":\"" + String(temp_event.temperature) +
               "\",\"h\":\"" + String(humidity_event.relative_humidity) +
               "\",\"p\":\"" + String(pressure_event.pressure) +
               "\"}";

  Serial.print(mqtt_status);
  Serial.print(" -> ");
  Serial.println(msg);
  
  mqttClient.beginMessage(mqtt_status, (unsigned long)msg.length());
  mqttClient.print(msg);
  mqttClient.endMessage();
  // for some reason the mqttClient library has .disconnect() as private
}


void loop() {
#if DEEPSLEEP != 1
  measure_and_send();
  // go to sleep to preserve power
  delay(SLEEP_INTERVAL * 1e3);
#endif
}
