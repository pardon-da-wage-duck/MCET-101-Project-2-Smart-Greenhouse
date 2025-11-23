/** This program
 *
 * Written by Group 73
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
#define SERIES_RESISITOR_THERM 10000
#define SERIES_RESISTOR_PHOTO 10000

// Digital Sensors
#define PIRSENSOR 4
#define DHTPIN 2      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

// defining pins for actuators
// uses PWM pins 3, 5, 6, 9-11
#define MOTOR1 5
#define MOTOR2 6
#define LEDS 9

float waterLevelSensorLength = 1.605; // inches

DHT dht(DHTPIN, DHTTYPE); // define dht11 object

// function declarations
float readThermistor(int, bool);
float readThermistor(int, int, bool);
float readPhotoresistor(int, int);
float readWaterLevel(int, float);

void setup()
{
  Serial.begin(115200);

  dht.begin();

  // init sensors and actuatorS
  pinMode(WATERLEVEL, INPUT);
  pinMode(PHOTORESISTOR1, INPUT);
  pinMode(PHOTORESISTOR2, INPUT);
  pinMode(THERMISTOR1, INPUT);
  pinMode(THERMISTOR2, INPUT);
  pinMode(PIRSENSOR, INPUT);

  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(LEDS, OUTPUT);
}

void loop()
{
  delay(500);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float celsius = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float fahrenheit = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(celsius) || isnan(fahrenheit))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float humidityIndexF = dht.computeHeatIndex(fahrenheit, humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  float humidityIndexC = dht.computeHeatIndex(celsius, humidity, false);

  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(',');
  Serial.print(F("%  Temperature: "));
  Serial.print(celsius);
  Serial.print(',');
  Serial.print(F("°C "));
  Serial.print(fahrenheit);
  Serial.print(',');
  Serial.print(F("°F  Heat index: "));
  Serial.print(humidityIndexC);
  Serial.print(',');
  Serial.print(F("°C "));
  Serial.println(humidityIndexF);
  Serial.println(F("°F"));

  // Thermistor
  float therm1 = readThermistor(THERMISTOR1, SERIES_RESISITOR_THERM, false);
  float therm2 = readThermistor(THERMISTOR2, SERIES_RESISITOR_THERM, false);

  Serial.println(therm1);
  Serial.println(therm2);

  // Photoresistor
  float photo1 = readPhotoresistor(PHOTORESISTOR1, SERIES_RESISTOR_PHOTO);
  Serial.println(photo1);
  float photo2 = readPhotoresistor(PHOTORESISTOR2, SERIES_RESISTOR_PHOTO);
  Serial.println(photo2);

  // PIR Sensor
  float motionSensed = digitalRead(PIRSENSOR);
  Serial.println(motionSensed);

  // Read Water Level
  float waterlevel = readWaterLevel(WATERLEVEL, waterLevelSensorLength); // inches
  Serial.println(waterlevel);
}

// function definitions
float readThermistor(int pin, bool inCelsius)
{
  float thermistorReading = analogRead(pin) * (5 / 1023.0);
  float celsius = 24.239 * thermistorReading - 33.778;
  float fahrenheit = 43.63 * thermistorReading - 28.8;

  if (!inCelsius)
    return fahrenheit;
  else
    return celsius;
}

float readThermistor(int pin, int seriesResistor, bool inCelsius)
{
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  // Read the analog value from the thermistor
  float ThermistorVoltage = analogRead(pin) * (5 / 1023.0); // readVoltage(MyArduino, ThermistorPin) //Volts float
  float ThermistorResistance = seriesResistor * ((5 / ThermistorVoltage) - 1);

  // Apply Steinhart-Hart equation
  float logThermistorResistance = log(ThermistorResistance);
  float T = (1.0 / (c1 + c2 * logThermistorResistance + c3 * logThermistorResistance * logThermistorResistance * logThermistorResistance));
  float celsius = T - 273.15;
  float fahrenheit = (celsius * 9.0) / 5.0 + 32.0;

  if (!inCelsius)
    return fahrenheit;
  else
    return celsius;
}

float readPhotoresistor(int pin, int seriesResistor)
{
  float a = 6e6, b = -1.384;
  float ThermistorVoltage = analogRead(pin) * (5 / 1023.0);
  float ThermistorResistance = seriesResistor * ((5 / ThermistorVoltage) - 1);
  float lux = a * pow(ThermistorResistance, b);
  return lux;
}

float readWaterLevel(int pin, float maxHeight)
{
  float a = 1e-07;
  float b = 9.9077;
  float voltage = (analogRead(pin) * (5 / 1023.0));
  float percentage = a * exp(voltage * b);
  float height = maxHeight * percentage;
  if (percentage > 1)
    return maxHeight;
  return height;
}