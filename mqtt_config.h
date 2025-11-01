
// MQTT config parameters

#define MQTT_BROKER "mqtt.lan"
#define MQTT_PORT 1883

#define MQTT_PREFIX "ws-thp-v2"

#define DISCOVERY_PREFIX "homeassistant"

// every cycle we post state, every 10 cycles we post device autodiscover configs.
#define CONFIG_ANNOUNCE_INTERVAL 10

// sleep cycle length in seconds. Must be less than ~71mins (~4260) for esp deep sleep!
// this is how often we publish state
#define SLEEP_INTERVAL 5 * 60

// use deep sleep instead of modem sleep? must connect D0 to RST on esp8266
#define DEEPSLEEP 0
