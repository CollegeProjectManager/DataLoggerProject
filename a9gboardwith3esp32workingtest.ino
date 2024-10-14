void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // Use pins 16 (RX2) and 17 (TX2) for Serial2

  delay(2000);
  
  // Initialize A9G module
  Serial2.println("AT");
  delay(1000);
  Serial2.println("AT+CPIN?");
  delay(1000);

  // Set SMS text mode
  Serial2.println("AT+CMGF=1");
  delay(1000);
}

void loop() {
  // Replace 'YourMessage' with the actual message you want to send
  String message = "Hello from A9G!";
  String phoneNumber = "+919604263692";  // Replace with your desired phone number

  // Send SMS
  Serial2.print("AT+CMGS=\"");
  Serial2.print(phoneNumber);
  Serial2.println("\"");
  delay(1000);
  Serial2.print(message);
  delay(1000);
  Serial2.write(26);  // Ctrl+Z to send the message
  delay(1000);

  // Wait for the module to send the SMS
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }

  delay(5000);  // Wait for a few seconds before sending another SMS
}
