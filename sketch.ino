#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHTesp.h"
#include <Stepper.h> // Stepper motor library
#include <Arduino.h>

#define MOTOR1_PIN 19
#define MOTOR2_PIN 2

// WiFi Credentials
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// ThingSpeak API
const char* server = "http://api.thingspeak.com/update";
const char* apiKey = "LHZXRY1ELQVI0XGX"; // Replace with your ThingSpeak Write API Key

// DHT22 Sensor
const int DHT_PIN = 15; // Data pin connected to GPIO 15
DHTesp dhtSensor;

// MPU6050
Adafruit_MPU6050 mpu;

// PIR Sensor
const int PIR_PIN = 13;
const int LED_PIN = 12; // LED connected to GPIO 12

// Stepper Motors
const int stepsPerRevolution = 200;
Stepper stepper1(stepsPerRevolution, 18, 19, 21, 22); // Motor 1 pins
Stepper stepper2(stepsPerRevolution, 13, 12, 14, 27); // Motor 2 pins
int delaySpeed = 10;

void Task1(void *pvParameters) {
  while (true) {
    Serial.println("Task 1 running on Core: " + String(xPortGetCoreID()));
    // Read data from PIR sensor
  int motion = digitalRead(PIR_PIN);

  // Read data from MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Control LED based on motion
  if (motion == HIGH) {
    digitalWrite(LED_PIN, HIGH); // Turn LED on
  } else {
    digitalWrite(LED_PIN, LOW); // Turn LED off
  }
  TempAndHumidity dhtData = dhtSensor.getTempAndHumidity();
  float temperature = dhtData.temperature;
  float humidity = dhtData.humidity;

  
  // Log sensor readings to the Serial Monitor
  Serial.println("Temperature: " + String(temperature) + "°C");
  Serial.println("Humidity: " + String(humidity) + "%");
  Serial.println("Motion: " + String(motion));
  Serial.println("Accel X: " + String(accel.acceleration.x));
  Serial.println("Accel Y: " + String(accel.acceleration.y));
  Serial.println("Accel Z: " + String(accel.acceleration.z));
  Serial.println("Motor 1 Speed: " + String(100-delaySpeed) + " RPM");
  
  Serial.println("---");

  // Send data to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(server) + "?api_key=" + apiKey +
                 "&field1=" + String(temperature) +         // Field 1: Temperature
                 "&field2=" + String(humidity) +            // Field 2: Humidity
                 "&field3=" + String(motion) +              // Field 3: Motion detected
                 "&field4=" + String(accel.acceleration.x) + // Field 4: Acceleration X
                 "&field5=" + String(accel.acceleration.y) + // Field 5: Acceleration Y
                 "&field6=" + String(accel.acceleration.z);  // Field 6: Acceleration Z

    http.begin(url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("ThingSpeak Response: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error sending data to ThingSpeak: ");
      Serial.println(http.errorToString(httpResponseCode));
    }
    http.end();
  } else {
    Serial.println("WiFi not connected!");
  }

  }
  // Enter light sleep for 15 seconds
  esp_sleep_enable_timer_wakeup(15000000); // Time in microseconds
  Serial.println("Entering light sleep for 15 seconds...");
  esp_light_sleep_start();
  Serial.println("Woke up from light sleep!");
}

void Task2(void *pvParameters) {
  while (true) {
    Serial.println("Task 2 running on Core: " + String(xPortGetCoreID()));
    // Read data from DHT22
  
  for (int i = 0; i < 200; i++) {
    digitalWrite(MOTOR1_PIN, HIGH);
    digitalWrite(MOTOR1_PIN, LOW);
    TempAndHumidity dhtData = dhtSensor.getTempAndHumidity();
  float temperature = dhtData.temperature;

  delaySpeed = temperature == 0 ? 50:400/temperature;
  
  if (delaySpeed < 5 && delaySpeed>0){
    delaySpeed = 5;
  }
  if (delaySpeed > 50 || delaySpeed<0){
    delaySpeed = 50;
  }
    delay(delaySpeed);
  }
  for (int i = 0; i < 200; i++) {
    digitalWrite(MOTOR2_PIN, HIGH);
    digitalWrite(MOTOR2_PIN, LOW);
    TempAndHumidity dhtData = dhtSensor.getTempAndHumidity();
  float temperature = dhtData.temperature;

  delaySpeed = temperature == 0 ? 50:400/temperature;
  
  if (delaySpeed < 5 && delaySpeed>0){
    delaySpeed = 5;
  }
  if (delaySpeed > 50 || delaySpeed<0){
    delaySpeed = 50;
  }
    delay(delaySpeed);
  }
  
  }
}


void setup() {


   // // Initialize Stepper Motors
  stepper1.setSpeed(50); // Default speed
  stepper2.setSpeed(90); // Default speed
  Serial.begin(115200);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // Initialize DHT22 Sensor
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  // Initialize PIR Sensor
  pinMode(PIR_PIN, INPUT);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  

  // Create Task1 on Core 1
  xTaskCreatePinnedToCore(
      Task1,    // Function to execute
      "Task1",  // Name of the task
      10000,    // Stack size in bytes
      NULL,     // Task input parameter
      1,        // Priority of the task
      NULL,     // Task handle
      1);       // Core 1

  // Create Task2 on Core 0
  xTaskCreatePinnedToCore(
      Task2,    // Function to execute
      "Task2",  // Name of the task
      10000,    // Stack size in bytes
      NULL,     // Task input parameter
      1,        // Priority of the task
      NULL,     // Task handle
      0);       // Core 0

}




void loop() {
  // The loop can remain empty if tasks handle everything
}
