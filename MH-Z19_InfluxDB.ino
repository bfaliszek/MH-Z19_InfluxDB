#include <Arduino.h>
#include "src/MHZ19.h"                                     // https://github.com/WifWaf/MH-Z19 // updated - 10.01.2020
#include "src/InfluxDb.h"                               // https://github.com/tobiasschuerg/ESP8266_Influx_DB // updated - 10.01.2020

int READ_DATA_EVERY_SECONDS = 20; // default 20 sec
int SEND_DATA_EVERY_SECONDS = 60; // default 60 sec

#define INFLUXDB_HOST "XXX"
#define INFLUXDB_DATABASE "XXX"
#define INFLUXDB_USER "XXX"
#define INFLUXDB_PASS "XXX"
#define DEVICE_NAME "XXX"

// WiFi Config
#define WiFi_SSID "SSID"
#define WiFi_Password "PASSWORD"

#ifdef ARDUINO_ARCH_ESP32 // D17 - D16
#define RX_PIN 16                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 17                                          // Tx pin which the MHZ19 Rx pin is attached to
#else                     // D1 - D2
#define RX_PIN 4                                           // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 5                                           // Tx pin which the MHZ19 Rx pin is attached to
#endif

#ifdef ARDUINO_ARCH_ESP32
#include <HardwareSerial.h>
#include <WiFi.h>
#include <HTTPClient.h>
#else
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or Arduino package without SoftwareSerial support
#include <ESP8266WiFi.h>
#endif

#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library

#ifdef ARDUINO_ARCH_ESP32
HardwareSerial myMHZ19Serial(1);
#else
SoftwareSerial myMHZ19Serial(RX_PIN, TX_PIN);                   // (ESP8266 example) create device to MH-Z19 serial
#endif

unsigned long previousMillis_READING, previousMillis_SENDING = 0;
int ppm, TemperatureC = 0;

unsigned long READING_INTERVAL = READ_DATA_EVERY_SECONDS * 1000;
unsigned long SENDING_INTERVAL = SEND_DATA_EVERY_SECONDS * 1000;

Influxdb influx(INFLUXDB_HOST);

void take_measurements() {
  Serial.println("\nReading data...");
  ppm = myMHZ19.getCO2();                             // Request CO2 (as ppm)
  TemperatureC = myMHZ19.getTemperature();              // Request Temperature (as Celsius)

  if (ppm < 100 || ppm > 6000) {
    Serial.println("PPM not valid");
  } else {
    Serial.println("CO2: " + String(ppm) + " ppm");
  }
  Serial.println("Temperature: " + String(TemperatureC) + " C");
}

void send_data() {
  Serial.println("\nSending data...");
  InfluxData row(DEVICE_NAME);
  row.addValue("CO2", (int(ppm)));
  row.addValue("Temperature", (int(TemperatureC)));
  influx.write(row);
}

void setup()
{
  Serial.begin(115000);                                    // Device to serial monitor feedback

  WiFi.begin(WiFi_SSID, WiFi_Password);
  Serial.println();
  Serial.print("Waiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  influx.setDbAuth(INFLUXDB_DATABASE, INFLUXDB_USER, INFLUXDB_PASS);

#ifdef ARDUINO_ARCH_ESP32
  disableCore0WDT();
  //disableCore1WDT(); // ESP32-solo-1 so only CORE0!
  myMHZ19Serial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // (ESP32 Example) device to MH-Z19 serial start
#else
  myMHZ19Serial.begin(BAUDRATE);                                // (Uno example) device to MH-Z19 serial start
#endif
  myMHZ19.begin(myMHZ19Serial);                                 // *Serial(Stream) refence must be passed to library begin().

  myMHZ19.autoCalibration();                             // Turn auto calibration ON (OFF autoCalibration(false))
  //myMHZ19.autoCalibration(false);                          // Turn auto calibration OFF

  take_measurements();
}

void loop()
{
  yield();

  unsigned long currentMillis_READING = millis();
  if (currentMillis_READING - previousMillis_READING >= READING_INTERVAL) {
    take_measurements();
    previousMillis_READING = millis();
  }

  unsigned long currentMillis_SENDING = millis();
  if (currentMillis_SENDING - previousMillis_SENDING >= SENDING_INTERVAL) {
    send_data();
    previousMillis_SENDING = millis();
  }

}
