/**************************************************************************************
   Sketch to read any supported sensor and send the data via MQTT to the broker.
   Copyright 2018-2019 Aapo Rista / Vekotinverstas / Forum Virium Helsinki Oy
   MIT license

   The idea is to read all supported I2C and other sensors and send the data frequently
   to the server.
   This sketch supports (or will support) sensors listed below:
   - BME280 temp, humidity, pressure sensor
   - BME680 temp, humidity, pressure, VOC sensor
   - BH1750 LUX meter
   - APDS-9960 gesture / rgb light sensor
   - MLX90614 IR thermometer
   - Si7021 temperature / humidity sensor
   TODO:
   - a button or any device which creates interrupts
   - APDS-9960 gestures
   - Dallas DS18B20
 **************************************************************************************/

#include "settings.h"
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>            // Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     // Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// Sensor support libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_BME680.h"
#include <BH1750.h>               // https://github.com/claws/BH1750
#include <SparkFun_APDS9960.h>
#include "SparkFun_Si7021_Breakout_Library.h"  // https://github.com/sparkfun/Si7021_Breakout

// I2C settings
#define SDA     D2
#define SCL     D1

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  // Serial.println(payload);
}

// Define and set up all variables / objects
WiFiClient wifiClient;
WiFiManager wifiManager;
PubSubClient client(MQTT_SERVER, 1883, callback, wifiClient);
String mac_str;

/* Sensor variables */

// BME680 AQ sensor
Adafruit_BME680 bme680;
uint8_t bme680_ok = 0;
uint32_t bme680_lastRead = 0;

// BH1750 LUX sensor
BH1750 bh1750(0x23);
uint8_t bh1750_ok = 0;
uint32_t bh1750_lastRead = 0;
uint16_t bh1750_lux = -1;

// MLX90614 IR thermometer
uint8_t mlx90614_ok = 0;
uint32_t mlx90614_lastRead = 0;
float mlx90614_ambient_temp = -273.15;
float mlx90614_object_temp = -273.15;

// APDS9960 RGB and gesture sensor
SparkFun_APDS9960 apds9960 = SparkFun_APDS9960();
uint8_t apds9960_ok = 0;
uint32_t apds9960_lastRead = 0;

// Si7021 temperature and humidity sensor
Weather si7021;
uint8_t si7021_ok = 0;
uint32_t si7021_lastRead = 0;

// BME280 sensor
Adafruit_BME280 bme280;
uint8_t bme280_ok = 0;
uint32_t bme280_lastRead = 0;

float round_float(float val, int dec) {
  // Return val rounded to dec decimals
  return (int)(val * pow(10,dec) + 0.5) / 1.0 / pow(10,dec);
}

void MqttSetup() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi so no MQTT. Continuing.");
    return;
  }
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  clientName += mac_str;
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Connecting to ");
  Serial.print(MQTT_SERVER);
  Serial.print(" as ");
  Serial.println(clientName);

  if (client.connect((char*) clientName.c_str(), MQTT_USER, MQTT_PASSWORD)) {
    
    Serial.println("Connected to MQTT broker");
    Serial.print("Topic is: ");
    Serial.println(MQTT_TOPIC);

  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    // TODO: quit connecting after e.g. 20 seconds to enable standalone usage
    // abort();
  }
}

void setup() {
  mac_str = WiFi.macAddress();
  Wire.begin(SDA, SCL);
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  init_sensors();
  wifiManager.autoConnect(ESP_AP_NAME, ESP_AP_PW);
  MqttSetup();
}

void loop() {
  if (!client.loop()) {
    Serial.print("Client disconnected...");
    // TODO: increase reconnect from every loop() to every 60 sec or so
    MqttSetup();
    return;
  }
  read_sensors();
}

void init_sensors() {
  init_bme280();
  init_bme680();
  init_bh1750();
  init_mlx90614();
  init_apds9960();
  init_si7021();
}

void read_sensors() {
  read_bme280();
  read_bme680();  
  read_bh1750();  
  read_mlx90614();
  read_apds9960();
  read_si7021();
}

void init_bme280() {
  Serial.print(F("INIT BME280: "));
  if (bme280.begin(0x76)) {
    Serial.println(F("found"));
    bme280_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_bme280() {
  // Read BME280 if it has been initialised successfully and it is time to read it
  if ((bme280_ok == 1) && (millis() > (bme280_lastRead + BME280_SEND_DELAY))) {
    bme280_lastRead = millis();
    float humidity = round_float(bme280.readHumidity(), 1);
    float temp = round_float(bme280.readTemperature(), 1);
    float pres = round_float(bme280.readPressure() / 100.0F, 1);
    if (humidity > 0.0) {
      SendDataToMQTT("bme280", 
        "temp", temp,
        "humi", humidity,
        "pres", pres,
        "", 0
      );
    }
  }
}

void init_bme680() {
  Serial.print(F("INIT BME680: "));
  if (bme680.begin()) {
    Serial.println(F("found"));
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, BME680_HEATING_TIME); // 320*C for 150 ms, from settings.h
    bme680_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_bme680() {
  // Read BME680 if it has been initialised successfully and it is time to read it
  if ((bme680_ok == 1) && (millis() > (bme680_lastRead + BME680_SEND_DELAY))) {
    if (! bme680.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    } else {
      bme680_lastRead = millis();
      float humidity = round_float(bme680.humidity, 1);
      float temp = round_float(bme680.temperature, 2);
      float pres = round_float(bme680.pressure / 100.0F, 2);
      float gas = round_float(bme680.gas_resistance / 1000.0F, 1);
      
      SendDataToMQTT("bme680", 
        "temp", temp,
        "humi", humidity,
        "pres", pres,
        "gas", gas
      );
    }
  }
}

void init_bh1750() {
  Serial.print(F("INIT BH1750: "));
  bh1750.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  uint16_t lux = bh1750.readLightLevel();
  if (lux >= 0 && lux < 54612) {
    bh1750_ok = 1;
    Serial.print(F("found, "));
    Serial.print(lux);
    Serial.println(F(" lx"));
  } else {
    Serial.println(F("not found"));
  }
}

void read_bh1750() {
  // Read BH1750 if it has been initialised successfully and it is time to read it
  if ((bh1750_ok == 1) && (millis() > (bh1750_lastRead + BH1750_SEND_DELAY))) {
    bh1750_lastRead = millis();
    uint16_t lux = bh1750.readLightLevel();
    SendDataToMQTT("bh1750", 
      "lux", lux,
      "", 0,
      "", 0,
      "", 0
    );
  }
}

void init_mlx90614() {
  Serial.print(F("INIT MLX90614: "));
  mlx90614_object_temp = readObjectTempC(0x5A);
  if (mlx90614_object_temp >= -270 && mlx90614_object_temp < 1000) {
    mlx90614_ok = 1;
    Serial.print(F("found, "));
    Serial.print(mlx90614_object_temp);
    Serial.println(F(" 'C"));
  } else {
    Serial.println(F("not found"));
  }
}

void read_mlx90614() {
  // Read BH1750 if it has been initialised successfully and it is time to read it
  if ((mlx90614_ok == 1) && (millis() > (mlx90614_lastRead + MLX90614_SEND_DELAY))) {
    mlx90614_object_temp = round_float(readObjectTempC(0x5A), 1);
    mlx90614_ambient_temp = round_float(readAmbientTempC(0x5A), 1);
    mlx90614_lastRead = millis();
    SendDataToMQTT("mlx90614", 
      "obtemp", mlx90614_object_temp,
      "amtemp", mlx90614_ambient_temp,
      "", 0,
      "", 0
    );
  }
}

/*
 * NOTE: Arduino/libraries/SparkFun_APDS9960_RGB_and_Gesture_Sensor/src/SparkFun_APDS9960.h
 * conflicts with ESP8266WiFi and you must change NA_STATE --> N_A_STATE in SparkFun_APDS9960.h
 */
void init_apds9960() {
  Serial.print(F("INIT APDS9960: "));
  bool init_ok = apds9960.init(); // For some reason this may return false
  if (apds9960.enableLightSensor(false)) {
    Serial.println(F("found"));
      apds9960_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_apds9960() {
  if ((apds9960_ok == 1) && (millis() > (apds9960_lastRead + APDS9960_SEND_DELAY))) {
    apds9960_lastRead = millis();
    uint16_t r, g, b, a;
    if (  !apds9960.readAmbientLight(a) ||
          !apds9960.readRedLight(r) ||
          !apds9960.readGreenLight(g) ||
          !apds9960.readBlueLight(b) ) {
      Serial.println("Error reading light values");
      return;
    }
    SendDataToMQTT("apds9960",
      "r", r,
      "g", g,
      "b", b,
      "a", a
    );
  }
}

void init_si7021() {
  Serial.print(F("INIT Si7021: "));
  si7021.begin();
  float humidity = si7021.getRH();
  if (humidity > 0.0 && humidity <= 150.0) {
    // Serial.println(F("found"));
    si7021_ok = 1;
  } else {
    // Serial.println(F("not found"));
  }
}

void read_si7021() {
  if ((si7021_ok == 1) && (millis() > (si7021_lastRead + SI7021_SEND_DELAY))) {
    si7021_lastRead = millis();
    float humidity = round_float(si7021.getRH(), 1);
    float temp = round_float(si7021.getTemp(), 1);
    SendDataToMQTT("si7021",
      "temp", temp,
      "humi", humidity,
      "", 0,
      "", 0
    );
  }
}


void SendDataToMQTT(char const sensor[], 
                    char const type1[], float val1, 
                    char const type2[], float val2, 
                    char const type3[], float val3, 
                    char const type4[], float val4) {
  /**
   * Send data to the MQTT broker. Currently max 4 key/value pairs are supported. 
   * If you set typeX argument empty (""), if will be left out from the payload.
   */
  /* 
   *  NOTE!
   *  For some weird reason / bug MQTT topic + json message to be send can't exceed ~121 bytes!
   *  Check that message size + topic are at most 120 B.
   */
  // Serial.println("SendDataToMQTT start");
  // StaticJsonBuffer<512> jsonBuffer;
  uint16_t msg_len = 0;
  DynamicJsonBuffer jsonBuffer(512);
  char jsonChar[256];
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = sensor;
  root["mac"] = mac_str;
  JsonObject& data = root.createNestedObject("data");
  if (type1[0] != 0) { data[type1] = val1; }
  if (type2[0] != 0) { data[type2] = val2; }
  if (type3[0] != 0) { data[type3] = val3; }
  if (type4[0] != 0) { data[type4] = val4; }
  root.printTo(jsonChar);
  msg_len = strlen(MQTT_TOPIC) + strlen(jsonChar);
  Serial.print(msg_len);
  Serial.print("B ");
  Serial.print(MQTT_TOPIC);
  Serial.print(" ");
  Serial.println(jsonChar);
  if (msg_len > 120) {
    Serial.print("Warning: TOPIC + JSON > 120 bytes.");
  }
  client.publish(MQTT_TOPIC, jsonChar);
}

