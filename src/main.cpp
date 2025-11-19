/** This program
 *
 * Written by Kyle Jiang
 * Last updated: November 18, 2025
 */

// Dependent arduino libraries
#include <Arduino.h>
#include <DHT.h>

// Pin definitions

// Analog Sensors
#define WATERLEVEL A0
#define PHOTORESISTOR1 A1
#define PHOTORESISTOR2 A2
#define THERMISTOR1 A3
#define THERMISTOR2 A4

// Digital Sensors
#define PIRSENSOR 4
#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

// defining pins for actuators
// uses PWM pins 3, 5, 6, 9-11
#define MOTOR1 5
#define MOTOR2 6
#define LEDS 9

DHT dht(DHTPIN, DHTTYPE); // define dht11 object

// function declarations
void readDHDT();

void setup()
{
  Serial.begin(115200);

  dht.begin();
}

void loop()
{
  // put your main code here, to run repeatedly:
}

// function definitions

void readDHT()
{
  // Wait a few seconds between measurements.
  delay(2000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
}
