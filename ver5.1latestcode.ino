#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SD.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define DHTPIN1 4                // GPIO pin for the first DHT11 (ambient temperature)
#define DHTPIN2 15               // GPIO pin for the second DHT11 (motor temperature)
#define SPEED_SENSOR_PIN 2       // GPIO pin for Speed Sensor (D0 output of LM393)
#define SD_CS 5                  // GPIO pin for SD card CS
#define IR_SENSOR_PIN 12         // GPIO pin for IR sensor (collision detection)
#define DHTTYPE DHT11            // DHT sensor type

// Constants for Speed Calculation
#define PULSES_PER_REVOLUTION 20 // Number of pulses per full revolution of the wheel
#define WHEEL_DIAMETER_CM 3      // Diameter of the wheel in cm
#define WHEEL_DIAMETER_METERS (WHEEL_DIAMETER_CM / 100.0)  // Convert diameter to meters
#define WHEEL_CIRCUMFERENCE_METERS (PI * WHEEL_DIAMETER_METERS) // Circumference in meters

// DHT Sensors
DHT dht1(DHTPIN1, DHTTYPE);  // First DHT11 (ambient)
DHT dht2(DHTPIN2, DHTTYPE);  // Second DHT11 (motor)

// SD Card
const int chipSelect = SD_CS;

// Wi-Fi and Google Sheets URL
const char* ssid = "Virus";          // Replace with your Wi-Fi network name
const char* password = "Devendra@9625";  // Replace with your Wi-Fi password
const char* googleSheetsURL = "https://script.google.com/macros/s/AKfycbyLIj9dsP44zp7s-8izPENjaPVFBvh6u6yn6wZJjLcoK0BJcP_Y0yNhYqFNvrgjqwsEMA/exec";  // Replace with your Google Sheets web app URL

// I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MPU6050
MPU6050 mpu;
bool mpuStatus = false;  // Flag for MPU initialization status
int16_t ax, ay, az;  // Use int16_t for accelerometer values
long tiltStartMillis = 0;  // To store the start time of the tilt
bool accidentDetected = false;  // To track accident detection

// Timing control
unsigned long previousMillis = 0;
const long interval = 5000; // Total cycle time (5 seconds)

// Speed Sensor
volatile int pulseCount = 0;   // Number of pulses detected
unsigned long lastPulseTime = 0;
float speedRPM = 0.0;  // Speed in RPM
float speedKMH = 0.0; // Speed in km/h

// Collision detection variable
bool collisionDetected = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to be ready

  // Initialize I2C LCD
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data Logger");
  lcd.setCursor(0, 1);
  lcd.print("Project");
  delay(3000); // Display heading for 3 seconds
  lcd.clear();

  // Initialize DHT sensors
  dht1.begin();  // Ambient
  dht2.begin();  // Motor

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    lcd.setCursor(0, 0);
    lcd.print("SD Init Failed!");
    return;
  }
  Serial.println("SD card initialized.");
  lcd.setCursor(0, 0);
  lcd.print("SD Initialized");
  delay(2000);
  lcd.clear();

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi.");
  lcd.setCursor(0, 0);
  lcd.print("Wi-Fi Connected");
  delay(2000);
  lcd.clear();

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  mpuStatus = mpu.testConnection();
  if (mpuStatus) {
    Serial.println("MPU6050 connected.");
  } else {
    Serial.println("MPU6050 connection failed.");
    lcd.setCursor(0, 0);
    lcd.print("MPU Init Failed!");
    delay(2000);
    lcd.clear();
  }

  // Initialize IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);

  // Initialize Speed Sensor
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), pulseCounter, FALLING);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read ambient DHT11 sensor
  float humidity = dht1.readHumidity();
  float ambientTemperature = dht1.readTemperature();

  // Read motor DHT11 sensor
  float motorTemperature = dht2.readTemperature();

  // Check if any reads failed
  if (isnan(humidity) || isnan(ambientTemperature) || isnan(motorTemperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read IR sensor state for collision detection
  collisionDetected = digitalRead(IR_SENSOR_PIN) == LOW; // LOW indicates an obstacle detected

  // Read MPU6050 accelerometer data
  if (mpuStatus) {
    mpu.getAcceleration(&ax, &ay, &az);  // Get acceleration data in int16_t format
    float angleX = atan2(ax, az) * 180 / PI;  // Convert to angles
    float angleY = atan2(ay, az) * 180 / PI;

    // Check if the car is tilted left or right beyond a certain angle (e.g., >45 degrees)
    if (abs(angleX) > 45 || abs(angleY) > 45) {
      if (tiltStartMillis == 0) {
        tiltStartMillis = currentMillis;  // Record the time when the tilt started
      } else if (currentMillis - tiltStartMillis >= 2000) {
        accidentDetected = true;  // Tilt has persisted for more than 2 seconds
      }
    } else {
      tiltStartMillis = 0;  // Reset the tilt timer if the tilt goes back to normal
      accidentDetected = false;
    }
  }

  if (currentMillis - previousMillis >= interval) {
    // Calculate speed in RPM
    noInterrupts();
    int pulses = pulseCount;
    pulseCount = 0;
    interrupts();
    
    speedRPM = (pulses / (float)PULSES_PER_REVOLUTION) * 60.0; // Convert to RPM

    // Convert RPM to km/h
    speedKMH = (speedRPM * WHEEL_CIRCUMFERENCE_METERS * 60) / 1000; // Convert to km/h

    // Display ambient temperature, humidity, and motor temperature
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Amb: " + String(ambientTemperature) + "C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: " + String(humidity) + "%");
    delay(2000); // Display for 2 seconds

    // Display accident status, collision status, and speed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(accidentDetected ? "Accident Detected" : "No Accident");
    lcd.setCursor(0, 1);
    lcd.print(collisionDetected ? "Collision ALERT" : "NO COLLISION");
    delay(1000); // Display for 1 second

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Speed: " + String(speedKMH) + " km/h");
    delay(1000); // Display speed for 1 second

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Temp: " + String(motorTemperature) + "C");
    delay(1000); // Display motor temperature for 1 second

    // Log data to SD card
    File dataFile = SD.open("/datalog.txt", FILE_APPEND);
    if (dataFile) {
      String logMessage = String(ambientTemperature) + ", " + String(humidity) + ", " + String(motorTemperature) + ", ";
      logMessage += (accidentDetected ? "Accident Detected" : "No Accident") + String(", ");
      logMessage += (collisionDetected ? "Collision ALERT" : "NO COLLISION") + String(", ");
      logMessage += "Speed: " + String(speedKMH) + " km/h";
      dataFile.println(logMessage);
      dataFile.close(); // Close the file
      Serial.println("Data logged: " + logMessage);

      // Send data to Google Sheets
      sendDataToGoogleSheets(ambientTemperature, humidity, motorTemperature, accidentDetected ? "Accident Detected" : "No Accident", collisionDetected ? "Collision ALERT" : "NO COLLISION", speedKMH);
    } else {
      Serial.println("Failed to open SD card file for writing");
    }

    previousMillis = currentMillis;
  }
}

// Function to handle counting pulses from speed sensor
void pulseCounter() {
  pulseCount++;
}

// Function to send data to Google Sheets
void sendDataToGoogleSheets(float ambientTemp, float humidity, float motorTemp, String accidentStatus, String collisionStatus, float speedKMH) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(googleSheetsURL);  // Specify the URL

    // Create JSON payload
    String payload = "{\"ambientTemperature\":" + String(ambientTemp) +
                     ",\"humidity\":" + String(humidity) +
                     ",\"motorTemperature\":" + String(motorTemp) +
                     ",\"accidentStatus\":\"" + accidentStatus + "\"" +
                     ",\"collisionStatus\":\"" + collisionStatus + "\"" +
                     ",\"speed\":" + String(speedKMH) + "}";

    http.addHeader("Content-Type", "application/json");  // Set content type to JSON

    int httpResponseCode = http.POST(payload);  // Send the request

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Response from Google Sheets: " + response);
    } else {
      Serial.println("Error sending data to Google Sheets. HTTP response code: " + String(httpResponseCode));
    }

    http.end();  // Free resources
  } else {
    Serial.println("Wi-Fi not connected. Unable to send data.");
  }
}
