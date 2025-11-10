/*
  COMPLETE MERGED PROJECT CODE (v2)
  This code does everything:
  1. Reads Temperature & Humidity (DHT11).
  2. Controls Red/Green LEDs and a Buzzer for alerts.
  3. Connects to Wi-Fi.
  4. Gets real-world time from an NTP server.
  5. Displays all info on a local I2C LCD screen.
  6. Publishes all data (with timestamp) to MQTT for Node-RED.
*/

// --- 1. INCLUDE LIBRARIES ---
#include <WiFi.h>
#include <PubSubClient.h>     // For MQTT
#include "DHT.h"              // For Sensor
#include <Wire.h>             // For I2C
#include <LiquidCrystal_I2C.h> // For LCD
#include <WiFiUdp.h>          // For Time
#include <NTPClient.h>        // For Time

// --- 2. DEFINE ALL PINS & OBJECTS ---
// Sensor (matching your code)
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// LCD (matching your code)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Outputs (moved to new pins)
#define GREEN_LED_PIN 25
#define RED_LED_PIN   26
#define BUZZER_PIN    27

// --- 3. DEFINE ALERT THRESHOLDS ---
#define MIN_TEMP 18.0
#define MAX_TEMP 26.0
#define MAX_HUMIDITY 60.0

// --- 4. WI-FI & MQTT DETAILS ---
const char* ssid = "Wokwi-GUEST"; // Use "FAVOUR" for your real device
const char* password = "";        // Use "NOONEKNOWS" for your real device
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic = "hope/iot_project/student123";

WiFiClient espClient;
PubSubClient client(espClient);

// --- 5. TIME CLIENT (NTP) ---
WiFiUDP ntpUDP;
// Get time from NTP server (GMT/UTC)
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// --- 6. SETUP FUNCTION (runs once) ---
void setup() {
  Serial.begin(115200);
  
  // Set up all output pins
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Start Sensor
  dht.begin();

  // Start LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to WiFi...");

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  Serial.println("\nWiFi connected!");
  lcd.clear();
  lcd.print("WiFi Connected!");

  // Start MQTT
  client.setServer(mqtt_server, 1883);

  // Start Time Client
  timeClient.begin();
}

// --- 7. MQTT RECONNECT FUNCTION ---
void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    lcd.setCursor(0, 3);
    lcd.print("MQTT Connect...");
    
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      lcd.setCursor(0, 3);
      lcd.print("MQTT Connected ");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      lcd.setCursor(0, 3);
      lcd.print("MQTT FAILED    ");
      delay(5000);
    }
  }
}

// --- 8. MAIN LOOP (runs forever) ---
void loop() {
  // Check MQTT connection
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop(); // Listen for MQTT messages

  // Update the time
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime();

  // We will read sensors every 10 seconds (like your code)
  delay(10000); 

  // --- Read Data ---
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); // Reads in Celsius
  String alert_status = "normal"; 

  // Check if sensor read failed
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
    return; // Skip this loop
  }

  // --- Check Alert Logic ---
  if (temperature < MIN_TEMP || temperature > MAX_TEMP || humidity > MAX_HUMIDITY) {
    // ALERT STATE
    Serial.println("ALERT: Values are outside the normal range!");
    alert_status = "alert";
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    // NORMAL STATE
    Serial.println("Status: All values are normal.");
    alert_status = "normal";
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // --- Update LCD Display ---
  lcd.clear();
  // Line 0: Humidity
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(humidity, 1);
  lcd.print("%");
  // Line 1: Temperature
  lcd.setCursor(0, 1);
  lcd.print("Temp:     ");
  lcd.print(temperature, 1);
  lcd.print((char)223); // Degree symbol
  lcd.print("C");
  // Line 2: Time
  lcd.setCursor(0, 2);
  lcd.print("Time:     ");
  lcd.print(formattedTime);
  // Line 3: Status
  lcd.setCursor(0, 3);
  lcd.print("Status:   ");
  lcd.print(alert_status);


  // --- Publish Data to MQTT ---
  // Create JSON payload
  String payload = "{";
  payload += "\"temperature\":";
  payload += temperature;
  payload += ",";
  payload += "\"humidity\":";
  payload += humidity;
  payload += ",";
  payload += "\"status\":\"";
  payload += alert_status;
  payload += "\",";
  payload += "\"timestamp\":"; // Add the real timestamp
  payload += epochTime;
  payload += "}";

  // Send the message
  char payload_char[payload.length() + 1];
  payload.toCharArray(payload_char, payload.length() + 1);
  client.publish(mqtt_topic, payload_char);
  
  Serial.print("Published message: ");
  Serial.println(payload_char);
}
