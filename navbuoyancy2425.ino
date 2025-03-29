#include <Wire.h>

// Sensor constants
const uint8_t sensorAddress = 0x76; // Default I²C address for the Bar30
const uint8_t cmdPressure = 0x40;  // Command to request pressure data
const uint8_t cmdTemperature = 0x50; // Command to request temperature data

// Variables to store pressure and temperature
float pressure;
float temperature;
int seconds_to_bottom;

const int gpioPin = 9;  // Built-in LED on Arduino Nano


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize I²C communication
  Wire.begin();
  delay(100);

  Serial.println("Bar30 Sensor Initialized!");

  pinMode(gpioPin, OUTPUT);  // Set as output

  Serial.println("GPIO Initialized!");
}



void loop() {
  // Read pressure on deck
  pressure = readSensorData(cmdPressure);
    Serial.print("On Deck Pressure: ");
    Serial.print(pressure);
    Serial.println(" mbar");

    // Read temperature on deck
    temperature = readSensorData(cmdTemperature);
    Serial.print("On Deck Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

  // wait while on deck
  int pressure_change = 0;
  int previous_pressure = 9999;
  while (pressure_change < 1) {
    pressure = readSensorData(cmdPressure);
    Serial.print(pressure);
    Serial.print(" mbar;  ");
    delay(250);
    pressure_change = pressure - previous_pressure;
    previous_pressure = pressure;
  }

// TURN MOTOR ON HERE
// WAIT 1/2 second

// go down until pressure stops changing
  previous_pressure = 0;
  while (pressure_change > 0) {
    pressure = readSensorData(cmdPressure);
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" mbar");

    // Read temperature
    temperature = readSensorData(cmdTemperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    pressure_change = pressure - previous_pressure;

    previous_pressure = pressure;

    // Delay for readability msec
    delay(1000);
  }

//  digitalWrite(gpioPin, HIGH);  // Turn LED on
//  delay(500);
//  digitalWrite(gpioPin, LOW);   // Turn LED off
//  delay(500);
}

float readSensorData(uint8_t command) {
// setup to read pressure or temperature
  Wire.beginTransmission(sensorAddress);
  Wire.write(command);
  Wire.endTransmission();

  delay(10);

// actual read from the sensor
  Wire.requestFrom(sensorAddress, 3);
  if (Wire.available() == 3) {
    uint32_t data = 0;
    data |= Wire.read();
    data <<= 8;
    data |= Wire.read();
    data <<= 8;
    data |= Wire.read();

    
    if (command == cmdPressure) {
      return data / 10.0;
    } else if (command == cmdTemperature) {
      return data / 100.0; 
    }
  }
  return 0.0;
}
