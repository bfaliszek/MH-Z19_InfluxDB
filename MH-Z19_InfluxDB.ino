#include <Arduino.h>
#include "src/MHZ19.h"                                  // https://github.com/WifWaf/MH-Z19 // updated - 6.07.2020
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

#define MHZ19_AUTOCALIBRATION_ON // Turn ON or OFF(just delete or comment this line)

#ifdef ARDUINO_ARCH_ESP32
#define RX_PIN 16          // D16                        
#define TX_PIN 17          // D17
#else
#define RX_PIN 4           // D2                                     
#define TX_PIN 5           // D1                               
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

void mhz19_calibration() {
  unsigned long getDataTimer = 0;
  void verifyRange(int range);

  /*            ### setRange(value)###
         Basic:
         setRange(value) - set range to value (advise 2000 or 5000).
         setRange()      - set range to 2000.
         Advanced:
         Use verifyRange(int range) from this code at the bottom.
  */

  myMHZ19.setRange(5000);

  /*            ###calibrateZero()###
     Basic:
     calibrateZero() - request zero calibration
     Advanced:
     In Testing.
  */

  // myMHZ19.calibrateZero(); // Always OFF!!! Experimental!

  /*             ### setSpan(value)###
     Basic:
     setSpan(value) - set span to value (strongly recommend 2000)
     setSpan()      - set span to 2000;
  */

  myMHZ19.setSpan(2000);

  /*            ###autoCalibration(false)###
     Basic:
     autoCalibration(false) - turns auto calibration OFF. (automatically sent before defined period elapses)
     autoCalibration(true)  - turns auto calibration ON.
     autoCalibration()      - turns auto calibration ON.
     Advanced:
     autoCalibration(true, 12) - turns autocalibration ON and calibration period to 12 hrs (maximum 24hrs).
  */

  myMHZ19.autoCalibration(false);
}

void setup()
{
  Serial.begin(115000);                                    // Device to serial monitor feedback

#ifdef ARDUINO_ARCH_ESP32
  disableCore0WDT();
  //disableCore1WDT(); // ESP32-solo-1 so only CORE0!
  myMHZ19Serial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);  // (ESP32 Example) device to MH-Z19 serial start
#else
  myMHZ19Serial.begin(BAUDRATE);                                // (Uno example) device to MH-Z19 serial start
#endif
  myMHZ19.begin(myMHZ19Serial);                                 // *Serial(Stream) refence must be passed to library begin().

#ifdef MHZ19_AUTOCALIBRATION_ON
  myMHZ19.autoCalibration(true);                             // Turn auto calibration ON (OFF autoCalibration(false))
#else
  mhz19_calibration();                                       // Manual calibration
  //myMHZ19.autoCalibration(false);                          // Turn auto calibration OFF
#endif

  Serial.print("Auto Base Calibration(ABC) Status: "); myMHZ19.getABC() ? Serial.println("ON") :  Serial.println("OFF");

  take_measurements();

  WiFi.begin(WiFi_SSID, WiFi_Password);
  Serial.println();
  Serial.print("Waiting for WiFi... ");
  while (WiFi.status() != WL_CONNECTED) {
#ifdef ARDUINO_ARCH_ESP32
    WiFi.begin(WiFi_SSID, WiFi_Password);
#endif
    Serial.println(".");
    delay(500);
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  influx.setDbAuth(INFLUXDB_DATABASE, INFLUXDB_USER, INFLUXDB_PASS);

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

void verifyRange(int range)
{
  Serial.println("Requesting new range.");

  myMHZ19.setRange(range);                             // request new range write

  if (myMHZ19.getRange() == range)                     // Send command to device to return it's range value.
    Serial.println("Range successfully applied.");   // Success

  else
    Serial.println("Failed to apply range.");        // Failed
}
