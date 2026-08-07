// BasicRead.ino
// Example code for the NHY3274TH Color Sensor Library
// Reads and prints raw RGB and white data from the sensor

#include <Wire.h>
#include <NHY3274TH.h>  // Include the NHY3274TH library

// Create an instance of the NHY3274TH sensor
NHY3274TH sensor;

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  while (!Serial)
    ;  // Wait for the serial monitor to open

  // Initialize I2C communication
  Wire.begin();

  // Initialize the sensor
  if (!sensor.begin()) {
    Serial.println("Failed to initialize the NHY3274TH sensor!");
    while (1)
      ;  // Halt if the sensor is not detected
  }

  Serial.println("NHY3274TH sensor initialized successfully.");
}

void loop() {
  uint16_t red, green, blue, white;

  // Read raw RGB and white data from the sensor
  sensor.getRawData(&red, &green, &blue, &white);

  // Print the results to the serial monitor
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(" Green: ");
  Serial.print(green);
  Serial.print(" Blue: ");
  Serial.print(blue);
  Serial.print(" White: ");
  Serial.println(white);

  delay(1000);  // Wait for 1 second before reading again
}