/*
   Basic BME280 MQTT client -> publish sensor data to an MQTT server
*/

//
// This is using an esp8266 board!
//

// BME280
#include <Wire.h>
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
#include <ArduinoJson.h>
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

static long starttime = 0;
void weekly_reset() {
  if (starttime == 0)
    starttime = millis();
  else if ((millis() - starttime) > 1000 * 7 * 86400 + 4000 + random(4000)) { // slightly more than 1 week
    Serial.println("Long term reset timer expired.");
    delay(1000);
    ESP.restart();
  }
}

void setup() {
  // setup serial port
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  // setup sensor
  Serial.println("BME280 MQTT Client startup...");

  // bme280 module directly soldered on nodemcu, at pins d6(12), d5(14), gnd, 3v3!
  Wire.begin(12, 14);

  while (!bme.begin(0x76, &Wire)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(1e4);
  }

  Serial.println("BME280 Sensor ready");
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  // wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    // retry
    delay(1e3);
  }

  mac = WiFi.macAddress();
  devID = String(mqtt_prefix) + "-" + mac.substring(0,2) + mac.substring(3,5) + mac.substring(6,8)
            +mac.substring(9,11) + mac.substring(12,14) + mac.substring(15,17);

  Serial.println("WiFi connected, IP address: " + WiFi.localIP().toString());
  Serial.println("devID: " + devID);

#if DEEPSLEEP
  measure_and_send();
  // allow mqtt to deliver message
  delay(5);
  ESP.deepSleep(SLEEP_INTERVAL * 1e6);
#endif
}

static const String configTopic(String sensortype)
{
  return String(DISCOVERY_PREFIX) + "/sensor/" + devID + "/" + sensortype + "/config";
}

static void mqttPublish(String topic, String msg)
{
  mqttClient.beginMessage(topic, (unsigned long)msg.length());
  mqttClient.print(msg);
  mqttClient.endMessage();
}

static void measure_and_send() {
  String msg;
  JsonDocument ctbl;
  JsonDocument stbl;
  sensors_event_t temp_event, pressure_event, humidity_event;

#if !DEEPSLEEP
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

    ctbl["state_topic"] = mqtt_status;

    // temp config
    ctbl["device_class"] = "temperature";
    ctbl["name"] = devID + " - Temperature";
    ctbl["unique_id"] = devID + "-temperature";
    ctbl["unit_of_measurement"] = "°C";
    ctbl["value_template"] = "{{ value_json.t}}";
    serializeJson(ctbl, msg);
    mqttPublish(configTopic("temperature"), msg);

    // humidity config
    ctbl["device_class"] = "humidity";
    ctbl["name"] = devID + " - Humidity";
    ctbl["unique_id"] = devID + "-humidity";
    ctbl["unit_of_measurement"] = "%";
    ctbl["value_template"] = "{{ value_json.h}}";
    serializeJson(ctbl, msg);
    mqttPublish(configTopic("humidity"), msg);

    // pressure config
    ctbl["device_class"] = "pressure";
    ctbl["name"] = devID + " - Pressure";
    ctbl["unique_id"] = devID + "-pressure";
    ctbl["unit_of_measurement"] = "hPa";
    ctbl["value_template"] = "{{ value_json.p}}";
    serializeJson(ctbl, msg);
    mqttPublish(configTopic("pressure"), msg);
  };

  // only announce autodiscovery every CONFIG_ANNOUNCE_INTERVAL
  configAnnounce = (configAnnounce + 1) % CONFIG_ANNOUNCE_INTERVAL;

  // fetch sensor data
  bme_temp->getEvent(&temp_event);
  bme_humidity->getEvent(&humidity_event);
  bme_pressure->getEvent(&pressure_event);

  // and send
  stbl["t"] = String(temp_event.temperature);
  stbl["h"] = String(humidity_event.relative_humidity);
  stbl["p"] = String(pressure_event.pressure);
  serializeJson(stbl, msg);
  mqttPublish(mqtt_status, msg);

  Serial.println(mqtt_status + " -> " + msg);
}

void loop() {
  weekly_reset();
#if !DEEPSLEEP
  measure_and_send();
  // go to sleep to preserve power
  delay(SLEEP_INTERVAL * 1e3);
#endif
}
