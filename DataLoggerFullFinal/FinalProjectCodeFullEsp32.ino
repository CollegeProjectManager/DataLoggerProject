#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SD.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Pin Definitions
#define DHTPIN1 4                // GPIO pin for the first DHT11 (ambient temperature)
#define DHTPIN2 15               // GPIO pin for the second DHT11 (motor temperature)
#define SPEED_SENSOR_PIN 2       // GPIO pin for Speed Sensor (D0 output of LM393)
#define SD_CS 5                  // GPIO pin for SD card CS
#define COLLISION_SENSOR_PIN 12   // GPIO pin for collision detection
#define TILT_SENSOR_PIN 14      // GPIO pin for tilt sensor
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
const char* googleSheetsURL = "https://script.google.com/macros/s/AKfycbx6Xdm7nlp7SQRdsAejewPAPXm4OaT6O4QWm8F8vnkwyoOWEShM7eArQzfEbloN5WBr/exec";  // Replace with your Google Sheets web app URL

// I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Tilt Sensor
bool accidentDetected = false;  // To track accident detection
bool switchStatePrevious = HIGH;  // Track the previous state of the tilt sensor

// Timing control
unsigned long previousMillis = 0;
const long interval = 5000; // Total cycle time (5 seconds)

// Speed Sensor
volatile int pulseCount = 0;   // Number of pulses detected
unsigned long lastPulseTime = 0;
float speedRPM = 0.0;  // Speed in RPM
float speedKMH = 0.0; // Speed in km/h

// Function prototypes
void sendSMS(String message);
void IRAM_ATTR pulseCounter();  // Add declaration of the pulseCounter function

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Initialize A9G module
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // Use pins 16 (RX2) and 17 (TX2)
  delay(2000);
  
  // Set SMS text mode
  Serial2.println("AT+CMGF=1");
  delay(1000);

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

  // Initialize Switch Pins
  pinMode(COLLISION_SENSOR_PIN, INPUT);  // Collision sensor
  pinMode(TILT_SENSOR_PIN, INPUT);        // Tilt sensor

  // Initialize Speed Sensor
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), pulseCounter, FALLING);  // Attach the pulseCounter interrupt
}

void loop() {
  unsigned long currentMillis = millis();

  // Read ambient DHT11 sensor
  float ambientTemperature = dht1.readTemperature();

  // Read motor DHT11 sensor
  float motorTemperature = dht2.readTemperature();

  // Check if any reads failed
  if (isnan(ambientTemperature) || isnan(motorTemperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read the state of the collision sensor
  int collisionState = digitalRead(COLLISION_SENSOR_PIN);

  // Check for accident detection (using a digital tilt sensor)
  int tiltState = digitalRead(TILT_SENSOR_PIN);
  if (tiltState == LOW && switchStatePrevious == HIGH) {
    accidentDetected = true;  // Accident detected
    String messageToSend = "Accident Detected! Location: Unknown";  // Message to be sent via SMS
    sendSMS(messageToSend);  // Send SMS
  }
  switchStatePrevious = tiltState; // Update the previous state

  if (currentMillis - previousMillis >= interval) {
    // Calculate speed in RPM
    noInterrupts();
    int pulses = pulseCount;
    pulseCount = 0;
    interrupts();
    
    speedRPM = (pulses / (float)PULSES_PER_REVOLUTION) * 60.0; // Convert to RPM

    // Convert RPM to km/h
    speedKMH = (speedRPM * WHEEL_CIRCUMFERENCE_METERS * 60) / 1000; // Convert to km/h

    // Display ambient temperature
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Bat Temp: " + String(ambientTemperature) + "C");
    delay(2000); // Display for 2 seconds

    // Display accident status and speed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Accident: " + String(accidentDetected ? "Yes" : "No"));
    lcd.setCursor(0, 1);
    lcd.print("Speed: " + String(speedKMH) + " km/h");
    delay(2000); // Display for 2 seconds

    // Display motor temperature
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor: " + String(motorTemperature) + "C");
    delay(2000); // Display motor temperature for 2 seconds

    // Log to SD card and Google Sheets
    logData(ambientTemperature, motorTemperature, accidentDetected, collisionState, speedKMH);

    previousMillis = currentMillis;
  }
}

void logData(float ambientTemp, float motorTemp, bool accident, int collision, float speed) {
  // Log data to SD card
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Bat Temp: ");
    dataFile.print(ambientTemp);
    dataFile.print(", Motor Temp: ");
    dataFile.print(motorTemp);
    dataFile.print(", Accident: ");
    dataFile.print(accident ? "Yes" : "No");
    dataFile.print(", Collision: ");
    dataFile.print(collision == HIGH ? "No" : "Yes");
    dataFile.print(", Speed (km/h): ");
    dataFile.println(speed);
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  }

  // Log data to Google Sheets via HTTP POST request
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(googleSheetsURL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String postData = "ambientTemp=" + String(ambientTemp) +
                      "&motorTemp=" + String(motorTemp) +
                      "&accident=" + String(accident ? "Yes" : "No") +
                      "&collision=" + String(collision == HIGH ? "No" : "Yes") +
                      "&speed=" + String(speed);

    int httpResponseCode = http.POST(postData);

    if (httpResponseCode > 0) {
      Serial.println("Data sent to Google Sheets successfully.");
    } else {
      Serial.println("Error sending data to Google Sheets.");
    }

    http.end();
  }
}

void sendSMS(String message) {
  Serial2.print("AT+CMGS=\"+919604263692\"\r");  // Replace with your phone number
  delay(100);
  Serial2.print(message);
  delay(100);
  Serial2.write(26); // Send Ctrl+Z to indicate the end of the message
  delay(1000);  // Wait for the message to be sent
}

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}
