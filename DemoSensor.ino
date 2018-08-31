/**************************************************************************************
   Sketch to read any supported sensor and send the data via MQTT to the broker.
   Copyright 2018-2019 Aapo Rista / Vekotinverstas / Forum Virium Helsinki Oy
   MIT license

   The idea is to read all supported I2C and other sensors and send the data frequently
   to the server.
   This sketch supports (or will support) sensors listed below:
   - BME680 temp, humidity, pressure, VOC sensor
   - BME280 temp, humidity, pressure sensor
   - BH1750 LUX meter
   - APDS-9960 gesture / rgb light sensor
   - MLX90614 IR thermometer
   - Si7021 temperature / humidity sensor
   - a button or any device which creates interrupts
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
#include "Adafruit_BME680.h"
#include <BH1750.h>               // https://github.com/claws/BH1750

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

// bool wifiReconnect = false;

uint32_t lastMsgTime = 0;

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
  init_sensors();
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("Waiting 1 sec...");
  wifiManager.autoConnect();
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
  init_bme680();  
}

void read_sensors() {
  read_bme680();  
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
      SendDataToMQTT("bme680", 
        "temp", bme680.temperature,
        "humi", bme680.humidity,
        "pres", bme680.pressure / 100.0,
        "gas", bme680.gas_resistance / 1000.0
      );
      //create some variables to store the color data in
      /*
      Serial.print("Temp: ");
      Serial.print(val);
      Serial.print(" Humi: ");
      Serial.print(val2);
      Serial.print(" Gas: ");
      Serial.print(val3);
      Serial.print(" Pressure: ");
      Serial.println(bme.pressure / 100.0);
      */
    }
  }
}

void init_bh1750() {
  Serial.print(F("INIT BH1750: "));
  bh1750.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  uint16_t lux = bh1750.readLightLevel();
  if (lux >= 0 && lux < 54612) {
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
   *  NOTE!!!!!!!!!!
   *  For some weird reason / bug json message to be send can't exceed 106 bytes.
   *  Check that message size is at most 106 B.
   */
  // Serial.println("SendDataToMQTT start");
  // StaticJsonBuffer<512> jsonBuffer;
  DynamicJsonBuffer jsonBuffer(512);
  char jsonChar[256];
  JsonObject& root = jsonBuffer.createObject();
  root["sensor"] = sensor;
  root["mac"] = mac_str;
  JsonObject& data = root.createNestedObject("data");
  if (type1[0] != 0) { data[type1] = val1; }
  if (type1[1] != 0) { data[type2] = val2; }
  if (type1[2] != 0) { data[type3] = val3; }
  if (type1[3] != 0) { data[type4] = val4; }
  root.printTo(jsonChar);
  Serial.println(jsonChar);
  client.publish(MQTT_TOPIC, jsonChar);
}

