/*AgruminoPocketWeatherStation.ino - Sample project for Agrumino Lemon.
Created by gabriele.foddis@lifely.cc - Stay tuned on lifely.cc -
This sketch read and send all data in serial monitor.

Have fun !!!*/

#include <Wire.h>
#include <SPI.h>
#include <Agrumino.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define REFRESH_TIME 5000  //////ms
#define SEALEVELPRESSURE_HPA (1013.25)///change value
#define SERIAL_BAUD 115200
Adafruit_BME680 bme; // I2C connection
Agrumino agrumino;

void setup() {
  agrumino.setup();
  agrumino.turnBoardOn();
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  Serial.println("BME680 test in progress");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {

  boolean isAttachedToUSB =   agrumino.isAttachedToUSB();
  boolean isBatteryCharging = agrumino.isBatteryCharging();
  boolean isButtonPressed =   agrumino.isButtonPressed();
  float temperature =         agrumino.readTempC();
  unsigned int soilMoisture = agrumino.readSoil();
  unsigned int soilMoistureRaw = agrumino.readSoilRaw();
  float illuminance =         agrumino.readLux();
  float batteryVoltage =      agrumino.readBatteryVoltage();

  Serial.println("Data from Agrumino Lemon");
  Serial.println("isAttachedToUSB:   " + String(isAttachedToUSB));
  Serial.println("isBatteryCharging: " + String(isBatteryCharging));
  Serial.println("isButtonPressed:   " + String(isButtonPressed));
  Serial.println("Temperature:       " + String(temperature) + "°C");
  Serial.println("SoilMoisture:      " + String(soilMoisture) + "%");
  Serial.println("SoilMoisture:      " + String(soilMoistureRaw) + "Mv");
  Serial.println("illuminance :      " + String(illuminance) + " lux");
  Serial.println("batteryVoltage :   " + String(batteryVoltage) + " V");
  Serial.println("End");
  Serial.println("*********************");

  if (! bme.performReading()) {
    Serial.println("Error with perform reading :(");
    return;
  }

  Serial.println("Value from Bme680");
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" °C");
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" mt");
  Serial.println("End");
  Serial.println("*********************");
  delay(REFRESH_TIME);
}
