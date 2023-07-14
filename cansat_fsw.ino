#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BME680.h";

// for quectel l89
#define rxGPS 3
#define txGPS 2
SoftwareSerial gpsSerial(rxGPS, txGPS);
TinyGPSPlus gps;


// Bme68x bme;
Adafruit_BME680 bme;

// Adafruit BNO
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define SEALEVELPRESSURE_HPA (1013.25)

// Temperature = 28.28 *C
// Pressure = 911.03 hPa
// Humidity = 67.11 %
// Gas = 13.47 KOhms
// Approx. Altitude = 887.82 m
// LAT: 13.118586
// LONG: 77.615398
// X: 178.8125
// Y: 5.0000
// Z: 106.1250

// related to bme688 sensor
float temp = 0.0, pressure = 0.0, humidity = 0.0, gas = 0.0, altitude = 0.0;
// related to bno055 sensor
float x = 0.0, y = 0.0, z = 0.0;
// related to quectel l89
float latitude = 0.0, longitude = 0.0;

// deliminator 
String del = ",";


void setup() {
  Serial.begin(115200);   // connect serial
  gpsSerial.begin(9600);  // connect gps sensor


  // configuration related to BME688
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1)
      ;
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  // config related to bno055

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
}



void loop() {

  // BNO 055 READING STARTS

  sensors_event_t event;
  bno.getEvent(&event);

  // readings for bno055
  x = event.orientation.x;
  y = event.orientation.y;
  z = event.orientation.z;

  // BME 688 READING STARTS

  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }

  delay(50);
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  temp = bme.temperature;                             // in *c
  pressure = bme.pressure;                            // in hPa
  humidity = bme.humidity / 100.0;                    // in %
  gas = bme.gas_resistance / 1000.0;                  // in KOhms
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);  // in m

  // QUECTEL L89 READING STARTS

  while (gpsSerial.available())  // check for gps data
  {
    if (gps.encode(gpsSerial.read()))  // encode gps data
    {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }

  // char output = x + "," + y;

  Serial.println(x + del + y + del + z + del + temp + del + pressure + del + humidity + del + gas + del + altitude + del + latitude + del + longitude);
  // Serial.println(output)
}