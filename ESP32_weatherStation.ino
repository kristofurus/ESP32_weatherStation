#include <Arduino.h>
// I2C library
#include <Wire.h>
// SPI library
#include <SPI.h>

// SHT31
// https://github.com/adafruit/Adafruit_SHT31
#include "Adafruit_SHT31.h"

// DHT11 & DHT22
// https://github.com/adafruit/DHT-sensor-library
#include "DHT.h"

// DS18B20
#include <OneWire.h>
// https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DallasTemperature.h>

// MCP9808
// https://github.com/adafruit/Adafruit_MCP9808_Library
#include "Adafruit_MCP9808.h"

// BME280
// https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// AHT10
// https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_AHTX0.h>

// WiFi and http protocol communication
#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

// ------------------------------------------- DEVICE PARAMS -------------------------------------------//
#define DEVICE_NAME "ESP32_OUTSIDE"
#define DEVICE_ID 0

// ------------------------------------------- USED PINS -------------------------------------------//
// Communication pins
// GPIO22 - SCL   (I2C)
// GPIO21 - SDA   (I2C)
// GPIO27 - One wire bus
// GPIO26 - DHT11
// GPIO25 - DHT22
#define ONE_WIRE_BUS 27
#define DHT11_PIN 26
#define DHT22_PIN 25

// ESP32 status pins
// GPIO2  - LED   (SEND STATUS)
#define LED_PIN 2
// GPIO34 - V_bat
#define BAT_PIN 34

// ------------------------------------------- SENSORS DECLARATION -------------------------------------------//

// I2C sensors addresses
#define SHT31_I2C_ADDRESS   0x44
#define MCP9808_I2C_ADDRESS 0x18
#define BME280_I2C_ADDRESS  0x76
#define AHT10_I2C_ADDRESS   0x38

const float ERROR_VALUE = -500.0f;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

Adafruit_MCP9808 mcp9808 = Adafruit_MCP9808();

Adafruit_BME280 bme280;

Adafruit_AHTX0 aht10;

DHT dht11(DHT11_PIN, DHT11);

DHT dht22(DHT22_PIN, DHT22);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// ------------------------------------------- GLOBAL PARAMETERS -------------------------------------------//

const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
const char* serverName = SERVER_ADDRESS;

// ------------------------------------------- SENSORS STATUSES -------------------------------------------//
bool status_sht31;
bool status_mcp9808;
bool status_bme280;
bool status_aht10;
bool status_dht11;
bool status_dht22;
bool status_ds18b20;

// send data every 5 minutes
unsigned long timerDelay = 5*60*1000;

void setup() {
  
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  // initiate all sensors
  // TODO: For now DHT11 and DHT22 just assumes that they work. Should add some kind of check if they are really ok. Maybe with expectPulse
  status_sht31 = sht31.begin(SHT31_I2C_ADDRESS);
  status_mcp9808 = mcp9808.begin(MCP9808_I2C_ADDRESS);
  status_bme280 = bme280.begin(BME280_I2C_ADDRESS);
  status_aht10 = aht10.begin(&Wire, 0, AHT10_I2C_ADDRESS); //I2C, sensorID, sensorAddress
  dht11.begin();
  status_dht11 = true;
  dht22.begin();
  status_dht22 = true;
  ds18b20.begin();
  status_ds18b20 = ds18b20.getDS18Count() > 0 ? 1 : 0;

  // TODO: if status of a sensor != 0 report to server

  // configure sensors
  if(status_mcp9808) {
    mcp9808.setResolution(3); // set temp resolution to 0.0625°C
  }

  // declaration LED pin as output
  // pinMode(LED_PIN,OUTPUT);
 
}

void loop() {

  // sensors read values
  float t_sht31, t_mcp9808, t_bme280, t_aht10, t_dht11, t_dht22, t_ds18b20;
  float h_sht31,            h_bme280, h_aht10, h_dht11, h_dht22;
  float                     p_bme280;

  if (status_sht31) {
    t_sht31 = sht31.readTemperature();
    h_sht31 = sht31.readHumidity();

    Serial.print("SHT31 temp = ");
    Serial.print(t_sht31);
    Serial.print(", hum= ");
    Serial.println(h_sht31);
  }

  if (status_mcp9808) {
    // wake up sensor, read temperature and put it back to sleep
    mcp9808.wake();
    t_mcp9808 = mcp9808.readTempC();
    mcp9808.shutdown_wake(1);

    Serial.print("MCP9808 temp = ");
    Serial.println(t_mcp9808);
  }

  if (status_bme280) {
    t_bme280 = bme280.readTemperature();
    h_bme280 = bme280.readHumidity();
    p_bme280 = bme280.readPressure() / 100.0F; // preasure in hPa

    Serial.print("BME280 temp = ");
    Serial.print(t_bme280);
    Serial.print(", hum = ");
    Serial.print(h_bme280);
    Serial.print(", preas = ");
    Serial.println(p_bme280);
  }

  if (status_aht10) {
    sensors_event_t t_aht10_event, h_aht10_event;
    aht10.getEvent(&h_aht10_event, &t_aht10_event);
    t_aht10 = t_aht10_event.temperature;
    h_aht10 = h_aht10_event.relative_humidity;

    Serial.print("AHT10 temp = ");
    Serial.print(t_aht10);
    Serial.print(", hum = ");
    Serial.println(h_aht10);
  }

  if (status_dht11) {
    t_dht11 = dht11.readTemperature();
    h_dht11 = dht11.readHumidity();

    Serial.print("DHT11 temp = ");
    Serial.print(t_dht11);
    Serial.print(", hum = ");
    Serial.println(h_dht11);
  }

  if (status_dht22) {
    h_dht22 = dht22.readHumidity();
    t_dht22 = dht22.readTemperature();

    Serial.print("DHT22 temp = ");
    Serial.print(t_dht22);
    Serial.print(", hum = ");
    Serial.println(h_dht22);
  }
  
  if (status_ds18b20) {
    // temperature 85*C is reset temperature value. To read correct temp use 
    // requestTemperatures() method
    ds18b20.requestTemperatures();
    t_ds18b20 = ds18b20.getTempCByIndex(0);

    Serial.print("DS18B20 temp = ");
    Serial.println(t_ds18b20);
  }

  // send data
  if(WiFi.status() == WL_CONNECTED){
    WiFiClient client;
    HTTPClient http;

    http.begin(client, serverName);
    http.addHeader("Content-Type", "application/json");

    String httpRequestData = "{";
    httpRequestData += "\"device_name\":\"" + String(DEVICE_NAME) +"\",";
    httpRequestData += "\"device_id\":" + String(DEVICE_ID) +","; 

    if(status_sht31) {
      httpRequestData += "\"sht31_temp\":" + String(t_sht31) + ",";
      httpRequestData += "\"sht31_hum\":" + String(h_sht31) + ",";
    } else {
      httpRequestData += "\"sht31_temp\":" + String("null") + ",";
      httpRequestData += "\"sht31_hum\":" + String("null") + ",";
    }

    if(status_mcp9808) {
      httpRequestData += "\"mcp9808_temp\":" + String(t_mcp9808) + ",";
    } else {
      httpRequestData += "\"mcp9808_temp\":" + String("null") + ",";
    }
    
    if(status_bme280) {
      httpRequestData += "\"bme280_temp\":" + String(t_bme280) + ",";
      httpRequestData += "\"bme280_hum\":" + String(h_bme280) + ",";
      httpRequestData += "\"bme280_pres\":" + String(p_bme280) + ",";
    } else {
      httpRequestData += "\"bme280_temp\":" + String("null") + ",";
      httpRequestData += "\"bme280_hum\":" + String("null") + ",";
      httpRequestData += "\"bme280_pres\":" + String("null") + ",";
    }

    if(status_aht10) {
      httpRequestData += "\"aht10_temp\":" + String(t_aht10) + ",";
      httpRequestData += "\"aht10_hum\":" + String(h_aht10) + ",";
    } else {
      httpRequestData += "\"aht10_temp\":" + String("null") + ",";
      httpRequestData += "\"aht10_hum\":" + String("null") + ",";
    }

    if(status_dht11 && !isnan(t_dht22) && !isnan(h_dht11)) {
      httpRequestData += "\"dht11_temp\":" + String(t_dht11) + ",";
      httpRequestData += "\"dht11_hum\":" + String(h_dht11) + ",";
    } else {
      httpRequestData += "\"dht11_temp\":" + String("null") + ",";
      httpRequestData += "\"dht11_hum\":" + String("null") + ",";
    }

    if(status_dht22 && !isnan(t_dht22) && !isnan(h_dht22)) {
      httpRequestData += "\"dht22_temp\":" + String(t_dht22) + ",";
      httpRequestData += "\"dht22_hum\":" + String(h_dht22) + ",";
    } else {
      httpRequestData += "\"dht22_temp\":" + String("null") + ",";
      httpRequestData += "\"dht22_hum\":" + String("null") + ",";
    }

    if(status_ds18b20) {
      httpRequestData += "\"ds18b20_temp\":" + String(t_ds18b20) + "}";
    } else {
      httpRequestData += "\"ds18b20_temp\":" + String("null") + "}";
    }
    
    Serial.println(httpRequestData);
    int httpResponseCode = http.POST(httpRequestData);

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    if(httpResponseCode == 200){
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
    }   
    http.end();
    delay(timerDelay);
  } else {
    Serial.println("WiFi Disconnected");
  }
}
