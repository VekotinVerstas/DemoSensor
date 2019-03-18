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
   - SDS011 PM2.5/PM10 (Particle matter) sensor
   - MHZ19 CO2 sensor 
   TODO:
   - a button or any device which creates interrupts
   - APDS-9960 gestures
   - Dallas DS18B20

  NOTE
  You must install libraries below using Arduino IDE's 
  Sketch --> Include Library --> Manage Libraries... command

   PubSubClient (version >= 2.6.0 by Nick O'Leary)
   ArduinoJson (version > 5.13 < 6.0 by Benoit Blanchon)
   WiFiManager (version >= 0.14.0 by tzapu)
   Adafruit Unified Sensor (version >= 1.0.2 by Adafruit)
   Adafruit BME280 Library
   Adafruit BME680 Library
   SparkFun APDS9960 RGB and Gesture Sensor
   SparkFun Si7021 Humidity and Temperature Sensor
   Nova Fitness Sds dust sensors library
   OneWire by Jim Studt, Tom Pollard etc.
   DallasTemperature by Miles Burton etc.
   
 **************************************************************************************/

#include "settings.h"             // Remember to copy settings-example.h to settings.h and check all values!
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
#include <Adafruit_BME680.h>
#include <SparkFun_APDS9960.h>
#include <SparkFun_Si7021_Breakout_Library.h>  // https://github.com/sparkfun/Si7021_Breakout
#include <SdsDustSensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "src/BH1750.h"           // https://github.com/claws/BH1750
#include "src/mhz19.h"

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
unsigned long lastMqttMsgTime;
unsigned long mqttConnRetries = 0;

/* Sensor variables */

// BME680 AQ sensor
Adafruit_BME680 bme680;
uint8_t bme680_ok = 0;
uint32_t bme680_lastRead = 0;
uint32_t bme680_lastSend = 0;
float bme680_lastTemp = -999;
float bme680_lastHumi = -999;
float bme680_lastPres = -999;
float bme680_lastGas = -999;

// BH1750 LUX sensor
BH1750 bh1750(0x23);
uint8_t bh1750_ok = 0;
uint32_t bh1750_lastRead = 0;
uint32_t bh1750_lastSend = 0;
uint16_t bh1750_lux = -1;
uint16_t bh1750_lastLux = -1;

// MLX90614 IR thermometer
uint8_t mlx90614_ok = 0;
uint32_t mlx90614_lastRead = 0;
uint32_t mlx90614_lastSend = 0;
float mlx90614_ambient_temp = -273.15;
float mlx90614_object_temp = -273.15;
float mlx90614_ambient_lastTemp = -273.15;
float mlx90614_object_lastTemp = -273.15;

// APDS9960 RGB and gesture sensor
SparkFun_APDS9960 apds9960 = SparkFun_APDS9960();
uint8_t apds9960_ok = 0;
uint32_t apds9960_lastRead = 0;
uint32_t apds9960_lastSend = 0;
uint32_t apds9960_lastR = -999;
uint32_t apds9960_lastG = -999;
uint32_t apds9960_lastB = -999;
uint32_t apds9960_lastA = -999;

// Si7021 temperature and humidity sensor
Weather si7021;
uint8_t si7021_ok = 0;
uint32_t si7021_lastRead = 0;
uint32_t si7021_lastSend = 0;
float si7021_lastTemp = -999;
float si7021_lastHumi = -999;

// BME280 sensor
Adafruit_BME280 bme280;
uint8_t bme280_ok = 0;
uint32_t bme280_lastRead = 0;
uint32_t bme280_lastSend = 0;
float bme280_lastHumi = -999;
float bme280_lastTemp = -999;
float bme280_lastPres = -999;

// SDS011 PM sensor
// SDS011 Software serial settings
// TODO: Explicitly configure if is SDS011 present.
SdsDustSensor sds011(SDS011_RXPIN, SDS011_TXPIN);
uint8_t sds011_ok = 0;
uint32_t sds011_lastRead = 0;
uint32_t sds011_lastSend = 0;
float sds011_lastPM25 = -1.0;
float sds011_lastPM10 = -1.0;

SoftwareSerial mhz19(MHZ19_RXPIN, MHZ19_TXPIN);
uint8_t mhz19_ok = 0;
uint32_t mhz19_lastRead = 0;
uint32_t mhz19_lastSend = 0;
int mhz19_lastCO2 = -1.0;
int mhz19_lastTemp = -100.0;

// Dallas DS28B20 OneWire temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
uint8_t ds18b20_count = 0;
uint8_t ds18b20_ok = 0;
uint32_t ds18b20_lastRead = 0;
// Only 10 sensors supported
uint32_t ds18b20_lastSend[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float ds18b20_last[] = {-999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0, -999.0};
DeviceAddress tempDs18b20Address; // We'll use this variable to store a found device address

float round_float(float val, int dec) {
  // Return val rounded to dec decimals
  return (int)(val * pow(10,dec) + 0.5) / 1.0 / pow(10,dec);
}

float abs_diff(float a, float b) {
  float c = a - b;
  if (c < 0) {c = -c;}
  return c;
}

float log10_diff(float a, float b) {
  float log_a = log10(a);
  float log_b = log10(b);
  return abs_diff(log_a, log_b);
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
  lastMqttMsgTime = millis();
  init_sensors();
  char ap_name[30];
  sprintf(ap_name, "SensorDemo_%d", ESP.getChipId());
  Serial.print("AP name would be: ");
  Serial.println(ap_name);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(ap_name);
  MqttSetup();
}

void loop() {
  if (!client.loop()) {
    Serial.println("Client disconnected...");
    // TODO: increase reconnect from every loop() to every 60 sec or so
    mqttConnRetries++;
    if (mqttConnRetries > 10) {
      Serial.println("ESP.restart()");
      ESP.restart();
    }
    MqttSetup();
    return;
  }
  read_sensors();
  mqttConnRetries = 0;
}

void init_sensors() {
  init_bme280();
  init_bme680();
  init_bh1750();
  init_mlx90614();
  init_apds9960();
  init_si7021();
  init_sds011();
  init_mhz19();
  init_ds18b20();
}

void read_sensors() {
  read_pushButton();
  read_bme280();
  read_bme680();  
  read_bh1750();  
  read_mlx90614();
  read_apds9960();
  read_si7021();
  read_sds011();
  read_mhz19();
  read_ds18b20();
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
    float humi = bme280.readHumidity();
    float temp = bme280.readTemperature();
    float pres = bme280.readPressure() / 100.0F;
    // Send data only when it has changed enough or it is time to send it anyway    
    if (
        (millis() > (bme280_lastSend + SENSOR_SEND_MAX_DELAY)) ||
        (abs_diff(bme280_lastTemp, temp) > 0.2) ||
        (abs_diff(bme280_lastHumi, humi) > 1.0) ||
        (abs_diff(bme280_lastPres, pres) > 0.2)
    ) {
      bme280_lastSend = millis();
      SendDataToMQTT("bme280", 
        "temp", round_float(temp, 2),
        "humi", round_float(humi, 1),
        "pres", round_float(pres, 2),
        "", 0,
        -1
      );
      bme280_lastSend = millis();
      bme280_lastTemp = temp;
      bme280_lastHumi = humi;
      bme280_lastPres = pres;
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
      float temp = bme680.temperature;
      float humi = bme680.humidity;
      float pres = bme680.pressure / 100.0F;
      float gas = bme680.gas_resistance / 1000.0F;
      if (
          ((bme680_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
          (abs_diff(bme680_lastTemp, temp) > 0.2) ||
          (abs_diff(bme680_lastHumi, humi) > 1.0) ||
          (abs_diff(bme680_lastPres, pres) > 0.2) ||
          (abs_diff(bme680_lastGas, gas) > 3.0)
      ) {
        SendDataToMQTT("bme680",
          "temp", round_float(temp, 2),
          "humi", round_float(humi, 1),
          "pres", round_float(pres, 2),
          "gas", round_float(gas, 1),
          -1
        );
        bme680_lastSend = millis();
        bme680_lastTemp = temp;
        bme680_lastHumi = humi;
        bme680_lastPres = pres;
        bme680_lastGas = gas;
      }
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
    if (
        ((bh1750_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
        (log10_diff(bh1750_lastLux, lux) > 0.05)
    ) {  
      SendDataToMQTT("bh1750", 
        "lux", lux,
        "", 0,
        "", 0,
        "", 0,
        -1
      );
      bh1750_lastLux = lux;
      bh1750_lastSend = millis();
    }
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
  // Read MLX90614 if it has been initialised successfully and it is time to read it
  if ((mlx90614_ok == 1) && (millis() > (mlx90614_lastRead + MLX90614_SEND_DELAY))) {
    mlx90614_object_temp = readObjectTempC(0x5A);
    mlx90614_ambient_temp = readAmbientTempC(0x5A);
    mlx90614_lastRead = millis();
    if (
        ((mlx90614_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
        (abs_diff(mlx90614_object_lastTemp, mlx90614_object_temp) > 0.2) ||
        (abs_diff(mlx90614_ambient_lastTemp, mlx90614_ambient_temp) > 0.2) 
    ) {    
      SendDataToMQTT("mlx90614", 
        "obtemp", round_float(mlx90614_object_temp, 2),
        "amtemp", round_float(mlx90614_ambient_temp, 2),
        "", 0,
        "", 0,
        -1
      );
      mlx90614_lastSend = millis();
      mlx90614_object_lastTemp = mlx90614_object_temp;
      mlx90614_ambient_lastTemp = mlx90614_ambient_temp;
    }
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
    if (
        ((apds9960_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
        (abs_diff(apds9960_lastR, r) > 1.0) ||
        (abs_diff(apds9960_lastG, g) > 1.0) ||
        (abs_diff(apds9960_lastB, b) > 1.0) ||
        (abs_diff(apds9960_lastA, a) > 2.0)
    ) {    
      SendDataToMQTT("apds9960",
        "r", r,
        "g", g,
        "b", b,
        "a", a,
        -1
      );
      apds9960_lastSend = millis();
      apds9960_lastR = r;
      apds9960_lastG = g;
      apds9960_lastB = b;
      apds9960_lastA = a;
    }
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
    float humi = si7021.getRH();
    float temp = si7021.getTemp();
    if (
        ((si7021_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
        (abs_diff(si7021_lastTemp, temp) > 0.2) ||
        (abs_diff(si7021_lastHumi, humi) > 1.0)
    ) {    
      SendDataToMQTT("si7021",
        "temp", round_float(temp, 2),
        "humi", round_float(humi, 2),
        "", 0,
        "", 0,
        -1
      );
      si7021_lastSend = millis();
      si7021_lastTemp = temp;
      si7021_lastHumi = humi;
    }
  }
}

void init_sds011() {
  Serial.print(F("INIT sds011: "));
  sds011.begin();
  delay(1500); // Wait shortly to make sure SDS is responsive
  String undef = String("Mode: undefined");
  Serial.println(undef);
  if (undef == sds011.setContinuousWorkingPeriod().toString()) {
    Serial.println(F("not found"));
  } else {
    Serial.println(sds011.queryFirmwareVersion().toString()); // prints firmware version
    Serial.println(sds011.setActiveReportingMode().toString()); // ensures sensor is in 'active' reporting mode
    Serial.println(sds011.setContinuousWorkingPeriod().toString()); // ensures sensor has continuous working period - default but not recommended
    sds011_ok = 1;
  }
}

void read_sds011() {
  if ((sds011_ok == 1) && (millis() > (sds011_lastRead + SDS011_SEND_DELAY))) {
    sds011_lastRead = millis();
    PmResult pm = sds011.readPm();
    if (pm.isOk()) {
      float pm25 = pm.pm25;
      float pm10 = pm.pm10;
      Serial.print("PM2.5 = ");
      Serial.print(pm25);
      Serial.print(", PM10 = ");
      Serial.println(pm10);
      if (
          ((sds011_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
          (abs_diff(sds011_lastPM25, pm25) > 0.1) ||
          (abs_diff(sds011_lastPM10, pm10) > 0.1)
      ) {    
        SendDataToMQTT("sds011",
          "pm25", pm25,
          "pm10", pm10,
          "", 0,
          "", 0,
          -1
        );
        sds011_lastSend = millis();
        sds011_lastPM25 = pm25;
        sds011_lastPM10 = pm10;
      }
    }
  }
}

// MH-Z19
static bool exchange_command(uint8_t cmd, uint8_t data[], int timeout)
{
    // create command buffer
    uint8_t buf[9];
    int len = prepare_tx(cmd, data, buf, sizeof(buf));

    // send the command
    mhz19.write(buf, len);

    // wait for response
    long start = millis();
    while ((millis() - start) < timeout) {
        if (mhz19.available() > 0) {
            uint8_t b = mhz19.read();
            if (process_rx(b, cmd, data)) {
                return true;
            }
        }
    }
    return false;
}

static bool read_temp_co2(int *co2, int *temp)
{
    uint8_t data[] = {0, 0, 0, 0, 0, 0};
    bool result = exchange_command(0x86, data, 3000);
    if (result) {
        *co2 = (data[0] << 8) + data[1];
        *temp = data[2] - 40;
    }
    return result;
}


void init_mhz19() {
  Serial.print(F("INIT MH-Z19: "));
  mhz19.begin(9600);  
  delay(1500); // Wait shortly to make sure MH-Z19 is responsive
  if (read_temp_co2(&mhz19_lastCO2, &mhz19_lastTemp)) {
    mhz19_ok = 1;
    Serial.println(F("found"));
  } else {
    Serial.println(F("not found"));
  }
}

void read_mhz19() {
  if ((mhz19_ok == 1) && (millis() > (mhz19_lastRead + MHZ19_SEND_DELAY))) {
    mhz19_lastRead = millis();
    int CO2;
    int Temp;
    
    if (read_temp_co2(&CO2, &Temp)) {
      Serial.print("CO2: ");
      Serial.print(CO2, DEC);
      Serial.print(" TEMP: ");
      Serial.println(Temp, DEC);
      if (
          ((mhz19_lastSend + SENSOR_SEND_MAX_DELAY) < millis()) ||
          (abs_diff(mhz19_lastCO2, CO2) > 10) ||
          (abs_diff(mhz19_lastTemp, Temp) > 1)
      ) {    
        SendDataToMQTT("mhz19",
          "co2", CO2,
          "temp", Temp,
          "", 0,
          "", 0,
          -1
        );
        mhz19_lastSend = millis();
        mhz19_lastCO2 = CO2;
        mhz19_lastTemp = Temp;
      }
    }
  }
}



void init_ds18b20() {
  Serial.print(F("INIT DS18B20: "));
  ds18b20.begin();
  delay(500);//Wait for newly restarted system to stabilize
  ds18b20_count = ds18b20.getDeviceCount();
  if (ds18b20_count > 0) {
    Serial.print("Found ");
    Serial.print(ds18b20_count, DEC);
    Serial.println(" devices.");
    ds18b20_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
}

void read_ds18b20() {
  if ((ds18b20_ok == 1) && (millis() > (ds18b20_lastRead + DS18B20_SEND_DELAY))) {
    ds18b20_lastRead = millis();
    ds18b20_count = ds18b20.getDeviceCount();
    Serial.print("Requesting temperatures..."); 
    ds18b20.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");  
    for (uint8_t i=0; i<ds18b20_count; i++) {
      if(ds18b20.getAddress(tempDs18b20Address, i)) {
        // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
        ds18b20.setResolution(tempDs18b20Address, 11);
        float tempC = ds18b20.getTempC(tempDs18b20Address);
        int16_t id = tempDs18b20Address[6] * 256 + tempDs18b20Address[7];
        if (
            ((ds18b20_lastSend[i] + SENSOR_SEND_MAX_DELAY) < millis()) ||
            (abs_diff(ds18b20_last[i], tempC) > 0.2)
        ) {        
          SendDataToMQTT("ds18b20",
            "temp", tempC,
            "", 0,
            "", 0,
            "", 0,
            id
          );
          ds18b20_lastSend[i] = millis();
          ds18b20_last[i] = tempC;          
        }
      } else {
        Serial.print("No device found at ");
        Serial.print(i, DEC);
        Serial.println("");
      }
    }
  }
}


void SendDataToMQTT(char const sensor[], 
                    char const type1[], float val1, 
                    char const type2[], float val2, 
                    char const type3[], float val3, 
                    char const type4[], float val4,
                    int16_t sn) {
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
  if (sn >= 0) {
    root["sn"] = sn;
  }
  JsonObject& data = root.createNestedObject("data");
  if (type1[0] != 0) { data[type1] = val1; }
  if (type2[0] != 0) { data[type2] = val2; }
  if (type3[0] != 0) { data[type3] = val3; }
  if (type4[0] != 0) { data[type4] = val4; }
  root.printTo(jsonChar);
  msg_len = strlen(MQTT_TOPIC) + strlen(jsonChar);
  Serial.print(round_float((millis() / 1000.0), 2));
  Serial.print("s ");
  Serial.print(msg_len);
  Serial.print("B ");
  Serial.print(MQTT_TOPIC);
  Serial.print(" ");
  Serial.println(jsonChar);
  if (msg_len > 120) {
    Serial.println("Warning: TOPIC + JSON > 120 bytes.");
  }
  if (client.publish(MQTT_TOPIC, jsonChar)) {
    lastMqttMsgTime = millis();
  } else {
    Serial.println("Error: Publishing MQTT message failed.");
  }
}
