#include "BluetoothSerial.h"
#include <Arduino.h>

BluetoothSerial serialBT;

// Bluetooth signal store in this variable
char btSignal;

// Initial Speed
int Speed = 100;

// PWM pins for controlling the speed
int pwmPinA = 5;
int pwmPinB = 23;

// Motor controlling pins
int IN1 = 22;
int IN2 = 21;
int IN3 = 19;
int IN4 = 18;

void setup() {
  Serial.begin(115200);

  // Initialize Bluetooth
  serialBT.begin("Aslam Hossain YT");

  // Setup PWM pins (using analogWrite)
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);

  // Setup motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initial state of Car
  stop();
}

void loop() {
  while (serialBT.available()) {
    btSignal = serialBT.read();

    // Adjust speed based on received signal
    switch (btSignal) {
      case '0': Speed = 100; break;
      case '1': Speed = 110; break;
      case '2': Speed = 120; break;
      case '3': Speed = 130; break;
      case '4': Speed = 140; break;
      case '5': Speed = 150; break;
      case '6': Speed = 180; break;
      case '7': Speed = 200; break;
      case '8': Speed = 220; break;
      case '9': Speed = 240; break;
      case 'q': Speed = 255; break;
    }

    // Control motor direction based on received signal
    switch (btSignal) {
      case 'B': backward(); break;
      case 'F': forward(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      case 'S': stop(); break;
    }
  }
}

// Function to move backward
void backward() {
  analogWrite(pwmPinA, Speed);
  analogWrite(pwmPinB, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to move forward
void forward() {
  analogWrite(pwmPinA, Speed);
  analogWrite(pwmPinB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to turn left
void left() {
  analogWrite(pwmPinA, Speed);
  analogWrite(pwmPinB, Speed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Function to turn right
void right() {
  analogWrite(pwmPinA, Speed);
  analogWrite(pwmPinB, Speed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to stop the motors
void stop() {
  analogWrite(pwmPinA, 0); // Stop by setting PWM to 0
  analogWrite(pwmPinB, 0); // Stop by setting PWM to 0

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
