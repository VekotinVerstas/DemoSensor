// MQTT settings
#define MQTT_TOPIC "demosensor"
#define MQTT_SERVER "mqtt.example.org"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user_with_read-write_permission_to_topic"
#define MQTT_PASSWORD "mqtt_password"

#define SENSOR_SEND_MAX_DELAY 60000 // milliseconds, after this send data anyway even it has not changed at all

// Sensor settings
#define BME680_HEATING_TIME 150 // milliseconds
#define BME680_SEND_DELAY 10000 // milliseconds

#define BH1750_SEND_DELAY 1000  // milliseconds

#define MLX90614_SEND_DELAY 1000  // milliseconds

#define APDS9960_SEND_DELAY 1000  // milliseconds

#define SI7021_SEND_DELAY 1000  // milliseconds

#define BME280_SEND_DELAY 1000 // milliseconds

#define SDS011_SEND_DELAY 500 // milliseconds
#define SDS011_RXPIN D3
#define SDS011_TXPIN D4

#define DS18B20_SEND_DELAY 2000 // milliseconds
#define ONE_WIRE_BUS D3         // data pin (a 4.7-10K pull-up resistor is necessary, wemos mini has it built-in on D3, D4)
