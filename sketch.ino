#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// WiFi Credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// DHT22 Sensor
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// MPU6050
Adafruit_MPU6050 mpu;

// PIR Sensor
#define PIR_PIN 13

// Server endpoint
const char* serverUrl = "http://your-cloud-endpoint";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  dht.begin();
  mpu.begin();
  pinMode(PIR_PIN, INPUT);
}

void loop() {
  // Read DHT22 data
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Read PIR sensor data
  int motion = digitalRead(PIR_PIN);
  
  // Read MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    
    String postData = String("{\"temperature\":") + temperature +
                      ",\"humidity\":" + humidity +
                      ",\"motion\":" + motion +
                      ",\"acceleration_x\":" + a.acceleration.x +
                      ",\"acceleration_y\":" + a.acceleration.y +
                      ",\"acceleration_z\":" + a.acceleration.z + "}";

    int httpResponseCode = http.POST(postData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(response);
    } else {
      Serial.println("Error sending data");
    }
    http.end();
  }
  delay(2000);
}
