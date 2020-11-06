
// MQTT config parameters

#define MQTT_BROKER "10.0.0.1"
#define MQTT_PORT 1883
#define MQTT_TOPIC "bme280/0001"

// sleep cycle length in seconds. Must be less than ~71mins (~4260) for esp deep sleep!
#define SLEEP_INTERVAL 20 * 60

// use deep sleep instead of modem sleep? must connect D0 to RST on esp8266
#define DEEPSLEEP 1
