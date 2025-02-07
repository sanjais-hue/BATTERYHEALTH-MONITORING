#define BLYNK_TEMPLATE_ID "TMPL3HdDxLDHC"
#define BLYNK_TEMPLATE_NAME "Battery Health Monitoring"
#define BLYNK_AUTH_TOKEN "vuVL21v2l3AP41zZo60HFc00R190nyEV"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

#define VoltagePin 34 // GPIO34 on ESP32 for voltage measurement
#define LoadResistance 10.0 // Assume load resistance in ohms
#define Relay1 26  // GPIO26 for first relay control
#define Relay2 27  // GPIO27 for second relay control (temperature control)
#define DHTPIN 14  // GPIO14 for DHT11 data pin
#define DHTTYPE DHT11  // DHT 11 

DHT dht(DHTPIN, DHTTYPE);

char auth[] = BLYNK_AUTH_TOKEN;  // Blynk Auth Token
char ssid[] = "1234";         // WiFi SSID
char pass[] = "12345678";     // WiFi Password

float vOUT = 0.0;
float vIN = 0.0;
float current = 0.0;
float temperature = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(Relay1, OUTPUT); // Set Relay1 pin as output
  pinMode(Relay2, OUTPUT); // Set Relay2 pin as output
  digitalWrite(Relay1, LOW); // Ensure Relay1 is off at the beginning
  digitalWrite(Relay2, LOW); // Ensure Relay2 is off at the beginning
  dht.begin();
  Blynk.begin(auth, ssid, pass);
}

void loop() {
  Blynk.run();

  // Read voltage
  int value = analogRead(VoltagePin);
  vOUT = (value * 3.3) / 4095.0; // ESP32 ADC resolution is 12-bit (0-4095) and 3.3V reference
  vIN = vOUT / (R2 / (R1 + R2));
  current = vIN / LoadResistance; // Current calculation using Ohm's Law

  // Read temperature
  temperature = dht.readTemperature();

  // Check if the read failed and exit early (to try again).
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(vIN);
  lcd.print("V ");
  lcd.print("C:");
  lcd.print(current);
  lcd.print("A ");
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(temperature);
  lcd.print("C ");

  // Print to Serial Monitor
  Serial.print("Voltage : ");
  Serial.println(vIN);
  Serial.print("Current : ");
  Serial.println(current);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  // Send values to Blynk
  Blynk.virtualWrite(V1, vIN);         // Send voltage to Blynk app
  Blynk.virtualWrite(V2, current);     // Send current to Blynk app
  Blynk.virtualWrite(V3, temperature); // Send temperature to Blynk app

  // Check if voltage exceeds 9V
  if (vIN > 9.0) {
    Serial.println("Relay1 ON");
    digitalWrite(Relay1, HIGH); // Turn on relay 1
    Blynk.logEvent("voltage_overload", "Voltage overload!"); // Send notification to Blynk app
  } else {
    Serial.println("Relay1 OFF");
    digitalWrite(Relay1, LOW); // Turn off relay 1
  }

  // Check if temperature exceeds 31.5°C
  if (temperature > 31.5) {
    Serial.println("Relay2 ON");
    digitalWrite(Relay2, HIGH); // Turn on relay 2
    Blynk.logEvent("high_temperature", "High temperature! Cooling fan on."); // Send notification to Blynk app
  } else {
    Serial.println("Relay2 OFF");
    digitalWrite(Relay2, LOW); // Turn off relay 2
  }

  delay(1000); // Wait for 1 second
}
