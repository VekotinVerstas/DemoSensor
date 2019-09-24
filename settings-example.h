// AP name prefix, shown when sensor is connected to new WiFi
#define AP_PREFIX "AQBURK"
// BURK_ID - make this different for every device!!! Will be shown as a part of AP name
#define BURK_ID "800001"
// Software version is YYMMDD (float). Increase this when doing new revisions of code
#define DS_VERSION 190922

/************************************************************************************************
 * Connectivity settings
 * =====================
 * If you define BACKEND_URL (full URL including http:// prefix, HTTPS is not yet supported)
 * this sketch tries to POST data to BACKEND_URL 
 * 
 * If you define MQTT settings (specially MQTT_SERVER)
 * this sketch tries to connect MQTT broker and send the data there.
 * 
 * It is probably a very good idea to select just one of these, not both!
 * 
 * And if you can choose, take MQTT. It is much more efficient than HTTP in this case.
 * However, it may be easier to prototype with HTTP using a local http server (on your laptop or something).
 ************************************************************************************************/
// HTTP settings
/*
#define BACKEND_URL "http://10.255.255.171:8000/dump"
*/

// MQTT settings
/*
#define MQTT_TOPIC "demosensor"
#define MQTT_SERVER "mqtt.example.org"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user_with_read-write_permission_to_topic"
#define MQTT_PASSWORD "mqtt_password"
*/

#define MQTT_MAX_PACKET_SIZE 256 // For pubsubclient, default is 128 B
#define SENSOR_SEND_MAX_DELAY 60000 // milliseconds, after this send data anyway even it has not changed at all, good values are between 1-10 minutes
#define STATUS_SEND_DELAY 60000

// Sensor settings

// Buttons (digital HIGH / LOW)
// #define BUTTON_USE  // Comment out if there are no button-like sensors
#define PUSHBUTTON_1 D7
#define PUSHBUTTON_2 D8
#define PUSHBUTTON_SEND_DELAY 100

// Basic I2C sensors
#define BH1750_SEND_DELAY 1000  // milliseconds
#define MLX90614_SEND_DELAY 1000  // milliseconds
#define APDS9960_SEND_DELAY 1000  // milliseconds
#define SI7021_SEND_DELAY 1000  // milliseconds
#define BME280_SEND_DELAY 1000 // milliseconds

// SHT3x
#define SHT3x_SEND_DELAY 1000  // milliseconds
#define SHT3x_ADDR 0x45  // 0x45: wemos SHD30 sheild, 0x44 aliexpress SHT31
#define SHT3x_TYPE "sht30"  // Use sht30, sht31, sht35 etc. respectively

// BME680
#define BME680_HEATING_TIME 150 // milliseconds
#define BME680_SEND_DELAY 60000 // milliseconds

// Dallas DS18B20 settings
#define DS18B20_SEND_DELAY 2000 // milliseconds
#define ONE_WIRE_BUS D3         // data pin (a 4.7-10K pull-up resistor is necessary, wemos mini has it built-in on D3, D4)

// MHZ19 CO2 sensor
// #define MHZ19_USE  // Comment out if MHZ19 is not connected
#define MHZ19_SEND_DELAY 5000 // milliseconds
#define MHZ19_RXPIN D5
#define MHZ19_TXPIN D6

// K-type thermocouple sensor with maxNNNN amplifier
// #define THERMO_USE  // Comment out if MAX6675 or MAX31855 is not connected
#define THERMO_SEND_DELAY 2000 // milliseconds

// Nova SDS011 particulate matter (PM) sensor
// #define SDS011_USE // Comment out if SDS011 is not connected
#define SDS011_SEND_DELAY 500 // milliseconds
#define SDS011_RXPIN 14
#define SDS011_TXPIN 12
