
// MQTT config parameters

#define MQTT_BROKER "192.168.1.52"
#define MQTT_PORT 1883

#define MQTT_PREFIX "nodemcu-bme280"

#define DISCOVERY_PREFIX "homeassistant"

#define CONFIG_ANNOUNCE_INTERVAL 10

// sleep cycle length in seconds. Must be less than ~71mins (~4260) for esp deep sleep!
#define SLEEP_INTERVAL 5 * 60

// use deep sleep instead of modem sleep? must connect D0 to RST on esp8266
#define DEEPSLEEP 0
