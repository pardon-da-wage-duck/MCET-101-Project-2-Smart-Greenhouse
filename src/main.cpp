/** This program takes readings from multiple sensors and uses that data to control a set of motors and LEDs.
 * The data collected is also sent over serial to be read in MatLab where the data would be displayed in a GUI.
 * Written by Group 73
 * Last updated: November 24, 2025
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
float delayTime = 500;
float time = 0;

DHT dht(DHTPIN, DHTTYPE); // define dht11 object

// function declarations
void sendSerial(float);
float readThermistor(int, bool);
float readThermistor(int, int, bool);
float readPhotoresistor(int, int);
float readWaterLevel(int, float);
float calculateBrightness(float);
float fanPower(float);

void setup()
{
  Serial.begin(115200);

  dht.begin();

  // init sensors and actuatorS
  pinMode(WATERLEVEL, INPUT);
  pinMode(PHOTORESISTOR1, INPUT);
  pinMode(PHOTORESISTOR2, INPUT);
  pinMode(THERMISTOR1, INPUT);
  pinMode(PIRSENSOR, INPUT);

  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(LEDS, OUTPUT);
}

void loop()
{
  delay(delayTime);
  time += delayTime * 0.001; // keeps track of the runtime of the program in ms

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity(); // percentage

  // Internal Temperature of the greenhouse
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
  float heatIndexF = dht.computeHeatIndex(fahrenheit, humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  float heatIndexC = dht.computeHeatIndex(celsius, humidity, false);

  // Thermistor
  float therm1 = readThermistor(THERMISTOR1, SERIES_RESISITOR_THERM, false); // checks the temperature outside of the greenhouse

  // Photoresistor
  float photo1 = readPhotoresistor(PHOTORESISTOR1, SERIES_RESISTOR_PHOTO); // records the brightness within the greennhouse
  float photo2 = readPhotoresistor(PHOTORESISTOR2, SERIES_RESISTOR_PHOTO); // used to detect whether or not the lid of the greenhouse is closed
  float ledBrightness = calculateBrightness(photo1);
  analogWrite(LEDS, ledBrightness);
  ledBrightness = ledBrightness * (5 / 255.0); // volts

  // PIR Sensor
  float PIRReading = digitalRead(PIRSENSOR);

  // Read Water Level
  float waterlevel = readWaterLevel(WATERLEVEL, waterLevelSensorLength); // inches

  // controlling fan
  float fanPower1 = fanPower(heatIndexF);
  analogWrite(MOTOR1, fanPower1);
  analogWrite(MOTOR2, fanPower1);
  fanPower1 = fanPower1 * (5 / 255.0); // volts

  float data[12] = {humidity, celsius, fahrenheit, heatIndexC, heatIndexF, therm1, photo1, photo2, PIRReading, waterlevel, ledBrightness, fanPower1};

  for (unsigned int i = 0; i < sizeof(data) / sizeof(data[0]); i++)
  {
    sendSerial(data[i]);
  }
  Serial.println(time);
}

// function definitions
void sendSerial(float packet)
{
  Serial.print(packet);
  Serial.print(',');
}

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

float calculateBrightness(float surroundingBrightness)
{
  float brightness = 255 * (1 - (surroundingBrightness / 4000));
  return brightness;
}

float fanPower(float heatIndex)
{
  int maxTemp = 65; // fahrenheit
  if (heatIndex > maxTemp)
  {
    float power = map(heatIndex, 65, 100, 130, 255);
    return power;
  }
  else
    return 0.0;
}
