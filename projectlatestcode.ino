#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SD.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define DHTPIN 4                // GPIO pin for DHT11 (Ambient Temperature)
#define MOTOR_DHTPIN 14         // GPIO pin for DHT11 (Motor Temperature)
#define SPEED_SENSOR_PIN 2      // GPIO pin for Speed Sensor (D0 output of LM393)
#define SD_CS 5                 // GPIO pin for SD card CS
#define SWITCH_PIN 12
#define DHTTYPE DHT11         

// Constants for Speed Calculation
#define PULSES_PER_REVOLUTION 20 // Number of pulses per full revolution of the wheel
#define WHEEL_DIAMETER_CM 3     // Diameter of the wheel in cm
#define WHEEL_DIAMETER_METERS (WHEEL_DIAMETER_CM / 100.0)  // Convert diameter to meters
#define WHEEL_CIRCUMFERENCE_METERS (PI * WHEEL_DIAMETER_METERS) // Circumference in meters

// DHT Sensors
DHT dht(DHTPIN, DHTTYPE);        // Ambient Temperature
DHT motorDht(MOTOR_DHTPIN, DHTTYPE); // Motor Temperature

// SD Card
const int chipSelect = SD_CS;

// Wi-Fi and Google Sheets URL
const char* ssid = "Virus";          // Replace with your Wi-Fi network name
const char* password = "Devendra@9625";  // Replace with your Wi-Fi password
const char* googleSheetsURL = "https://script.google.com/macros/s/AKfycbxUFVO7nFWvPwXfcyvH05iKBYV6VSHp7Qck2tIbo1GB7B0thz47aYMzDTiGzl8IgDkP/exec";  // Replace with your Google Sheets web app URL

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
  dht.begin();          // Ambient Temperature
  motorDht.begin();     // Motor Temperature

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

  // Initialize Switch Pin
  pinMode(SWITCH_PIN, INPUT);

  // Initialize Speed Sensor
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), pulseCounter, FALLING);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read DHT11 sensors
  float ambientHumidity = dht.readHumidity();
  float ambientTemperature = dht.readTemperature();
  float motorHumidity = motorDht.readHumidity();
  float motorTemperature = motorDht.readTemperature();

  // Check if any reads failed
  if (isnan(ambientHumidity) || isnan(ambientTemperature) || isnan(motorHumidity) || isnan(motorTemperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read the state of the push switch (collision sensor)
  int switchState = digitalRead(SWITCH_PIN);

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

    // Display temperature, humidity, and speed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: " + String(ambientTemperature) + "C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: " + String(ambientHumidity) + "%");
    delay(2000); // Display for 2 seconds

    // Display accident status, switch status, and speed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(accidentDetected ? "Accident Detected" : "No Accident");
    lcd.setCursor(0, 1);
    lcd.print(switchState == HIGH ? "NO COLLISION" : "Collision ALERT");
    delay(1000); // Display for 1 second

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Speed: " + String(speedKMH) + " km/h");
    delay(1000); // Display speed for 1 second

    // Display motor temperature
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Temp: " + String(motorTemperature) + "C");
    delay(2000); // Display motor temperature for 2 seconds

    // Log data to SD card
    File dataFile = SD.open("/datalog.txt", FILE_APPEND);
    if (dataFile) {
      String logMessage = String(ambientTemperature) + ", " + String(ambientHumidity) + ", ";
      logMessage += (accidentDetected ? "Accident Detected" : "No Accident") + String(", ");
      logMessage += (switchState == HIGH ? "NO COLLISION" : "Collision ALERT") + String(", ");
      logMessage += "Speed: " + String(speedKMH) + " km/h" + String(", ");
      logMessage += "Motor Temp: " + String(motorTemperature) + " C";
      dataFile.println(logMessage);
      dataFile.close(); // Close the file
      Serial.println("Data logged: " + logMessage);

      // Send data to Google Sheets
      sendDataToGoogleSheets(ambientTemperature, ambientHumidity, accidentDetected ? "Accident Detected" : "No Accident", switchState == HIGH ? "NO COLLISION" : "Collision ALERT", speedKMH, motorTemperature);

      // Display "Data Logged" message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Data Logged");
      delay(1000); // Display for 1 second
    } else {
      Serial.println("Error opening datalog.txt");
    }

    // Update timing
    previousMillis = currentMillis;
  }
}

// Interrupt Service Routine for Speed Sensor
void pulseCounter() {
  pulseCount++;
}

// Function to send data to Google Sheets
void sendDataToGoogleSheets(float ambientTemp, float ambientHumidity, String accidentStatus, String collisionStatus, float speed, float motorTemp) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = String(googleSheetsURL) + "?ambientTemperature=" + ambientTemp +
                 "&ambientHumidity=" + ambientHumidity +
                 "&accidentStatus=" + accidentStatus +
                 "&collisionStatus=" + collisionStatus +
                 "&speed=" + speed +
                 "&motorTemperature=" + motorTemp;

    http.begin(url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Response: " + response);
    } else {
      Serial.println("Error code: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("Wi-Fi not connected.");
  }
}
