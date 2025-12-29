#include <WiFiS3.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- Pins ----------------
#define DHT_PIN     2
#define DHT_TYPE    DHT11
#define TDS_PIN     A0
#define PH_PIN      A1
#define RELAY1_PIN  8
#define RELAY2_PIN  9

// ---------- LCD (I2C) ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ------------- WiFi / MQTT ------------
const char* ssid        = "Airtel-MyWiFi-AMF-311WW-AA75";
const char* password    = "178237bb";

const char* mqtt_server = "broker.mqtt.cool";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "/sensordata";

WiFiClient wifiClient;
PubSubClient client(wifiClient);
DHT dht(DHT_PIN, DHT_TYPE);

// ---------- Timing / Relay logic ------
unsigned long lastMqttPublish          = 0;
const unsigned long mqttIntervalMs     = 1000;   // 1 s

unsigned long lastRelayToggle          = 0;
const unsigned long relayToggleIntervalMs = 60000x15; // 15 min

unsigned long lastLcdUpdate            = 0;
const unsigned long lcdIntervalMs      = 2000;   // 2 s

bool relayState = false;

// ------------ LCD helpers -------------
void initLCDLabels() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:      H:     ");
  lcd.setCursor(0, 1);
  lcd.print("TDS:    pH:   ");
}

void showStartupScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AI Robotics &");
  lcd.setCursor(0, 1);
  lcd.print("DnT Lab");
  delay(3000);
  initLCDLabels();
}

// NEW: show WiFi connecting status
void showWiFiConnecting() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connecting");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
}

// NEW: show WiFi OK + IP for a short time
void showWiFiOKWithIP() {
  IPAddress ip = WiFi.localIP();
  char ipStr[16];
  snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d",
           ip[0], ip[1], ip[2], ip[3]);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi OK");
  lcd.setCursor(0, 1);
  // lcd.print(ipStr);
  delay(3000);           // show for 3 seconds
  initLCDLabels();       // return to sensor layout
}

void updateLCD() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  float tds  = readTDS();
  float ph   = readPH() - 4.0;

  // Line 0: T and H
  lcd.setCursor(2, 0);
  if (!isnan(temp)) {
    if (temp < 10) lcd.print(" ");
    lcd.print(temp, 1);
    lcd.print(" ");
  } else {
    lcd.print("--  ");
  }

  lcd.setCursor(10, 0);
  if (!isnan(hum)) {
    if (hum < 10) lcd.print(" ");
    lcd.print(hum, 1);
    lcd.print(" ");
  } else {
    lcd.print("--  ");
  }

  // Line 1: TDS and pH
  lcd.setCursor(4, 1);
  
  
  lcd.print((int)tds);

  lcd.setCursor(12, 1);
  lcd.print("   ");
  lcd.setCursor(12, 1);
  lcd.print(ph, 1);
}

// ------------ Sensor helpers ----------
float readTDS() {
  int raw = analogRead(TDS_PIN);
  float voltage = (raw / 1023.0) * 5.0;
  return voltage * 500;
}

float readPH() {
  int raw = analogRead(PH_PIN);
  float voltage = (raw / 1023.0) * 5.0;
  float ph = 7.0 + (2.5 - voltage) * 3.0;
  return ph;
}

// ------------ WiFi helpers ------------
void connectWiFi() {
  int status = WL_IDLE_STATUS;

  // show connecting status on LCD
  showWiFiConnecting();

  while (status != WL_CONNECTED) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, password);
    delay(600);
  }

  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // show WiFi OK + IP briefly
  showWiFiOKWithIP();
}

// MQTT reconnect
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "UNO_R4_";
    clientId += String((uint32_t)millis(), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishSensorData() {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  float tds  = readTDS();
  float ph   = readPH();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT!");
    return;
  }

  IPAddress ip = WiFi.localIP();
  char ipStr[16];
  snprintf(ipStr, sizeof(ipStr), "%d.%d.%d.%d",
           ip[0], ip[1], ip[2], ip[3]);

  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"ip\":\"%s\",\"temperature\":%.2f,"
           "\"humidity\":%.2f,\"tds\":%.2f,\"ph\":%.2f,"
           "\"relay1\":\"%s\",\"relay2\":\"%s\"}",
           ipStr,
           temp, hum, tds, ph,
           relayState ? "ON" : "OFF",
           "OFF");

  Serial.print("Publishing: ");
  Serial.println(payload);

  if (client.publish(mqtt_topic, payload)) {
    Serial.println("Publish OK");
  } else {
    Serial.println("Publish FAILED");
  }
}

// ------------ Setup -------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  dht.begin();

  Wire.begin();
  lcd.init();
  lcd.backlight();
  showStartupScreen();

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
}

// ------------ Loop --------------------
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  unsigned long nowMs = millis();

  // relay1 toggle every 1 minute
  if (nowMs - lastRelayToggle >= relayToggleIntervalMs) {
    lastRelayToggle = nowMs;
    relayState = !relayState;
    digitalWrite(RELAY1_PIN, relayState ? HIGH : LOW);
    Serial.print("Relay 1 toggled, state: ");
    Serial.println(relayState ? "ON" : "OFF");
  }

  // MQTT publish every second
  if (nowMs - lastMqttPublish >= mqttIntervalMs) {
    lastMqttPublish = nowMs;
    publishSensorData();
  }

  // LCD update every 2 seconds
  if (nowMs - lastLcdUpdate >= lcdIntervalMs) {
    lastLcdUpdate = nowMs;
    updateLCD();
  }
}
