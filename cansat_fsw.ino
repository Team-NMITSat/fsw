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

// 0. Logging time
// 1. Relative height
// 2. Is in free fall (0 or 1)
// 3. Temperature
// 4. Atmospheric pressure
// 5. Pitch
// 6. Roll
// 7. Yaw
// 8. Acceleration in X
// 9. Y-axis acceleration
// 10. Z-acceleration

// related to bme688 sensor
float temp = 0.0, pressure = 0.0, humidity = 0.0, gas = 0.0, altitude = 0.0;
// related to bno055 sensor
float roll = 0.0, pitch = 0.0, heading = 0.0 , x_acc = 0.0 , y_acc = 0.0 , z_acc = 0.0;
// related to quectel l89
float latitude = 0.0, longitude = 0.0;

int hour = 0, minute = 0, second = 0, centisecond = 0;

int isFreeFall = 1;

// deliminator
String del = ",";


void setup() {
  Serial.begin(9600);     // connect serial
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
  roll = event.orientation.roll;
  pitch = event.orientation.pitch;
  heading = event.orientation.heading;

  imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  imu::Vector<3> 

  x_acc = li_ac.x();
  y_acc = li_ac.y();
  z_acc = li_ac.z();

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
      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();
    }
  }

// 10559,906.71,1,32.31,90898.00,-4.25,359.94,17.88,0.00,0.00

// 000,906.89,1,31.96,90896.00,-4.38,0.00,17.88,-0.76,-2.99,9.29,0.00,0.00


  Serial.println(String(hour) + String(minute) + String(second) + del + altitude + del + isFreeFall + del + temp + del + pressure + del + pitch + del + roll + del + heading + del + x_acc + del + y_acc + del + z_acc + del+ latitude + del + longitude);
  // Serial.println(output)
}
