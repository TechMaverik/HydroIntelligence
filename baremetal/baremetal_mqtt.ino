#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>                    // I2C support [web:34][web:38]
#include <LiquidCrystal_I2C.h>      // I2C LCD library [web:34][web:36]

// ---------- Pins ----------
#define DHT_PIN   4
#define DHT_TYPE  DHT11
#define TDS_PIN   34
#define PH_PIN    35

// ---------- WiFi ----------
const char* ssid     = "Ai Lab";
const char* password = "Welc0me@123";

// ---------- MQTT ----------
const char* mqtt_server = "broker.mqtt.cool";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "/sensor_data_stream";

// ---------- LCD (I2C) ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27 common, scan if needed [web:34][web:44]

// ---------- Globals ----------
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000;  // ms

// ---------- Helper: init LCD ----------
void setupLCD() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
}

// ---------- Helper: connect WiFi + show IP on LCD ----------
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  lcd.setCursor(0, 0);
  lcd.print("WiFi: ");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.print(".");
  }
  
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  // Display IP on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi OK");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString());
}

// ---------- Helper: connect MQTT ----------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "hydrointelligence";
    clientId += String(random(0xffff), HEX);

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

// ---------- Sensor read helpers ----------
float readTemperature() {
  float t = dht.readTemperature();
  if (isnan(t)) return -1000;
  return t;
}

float readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) return -1;
  return h;
}

float readTDS() {
  const uint8_t samples = 10;
  int raw = 0;
  for (uint8_t i = 0; i < samples; i++) {
    raw += analogRead(TDS_PIN);
    delay(10);
  }
  float avgRaw   = raw / (float)samples;
  float voltage  = avgRaw * (3.3 / 4095.0);
  float tdsValue = (133.42 * voltage * voltage * voltage
                   - 255.86 * voltage * voltage
                   + 857.39 * voltage) * 0.5;
  return tdsValue;
}

float readPH() {
  const uint8_t samples = 10;
  int raw = 0;
  for (uint8_t i = 0; i < samples; i++) {
    raw += analogRead(PH_PIN);
    delay(10);
  }
  float avgRaw  = raw / (float)samples;
  float voltage = avgRaw * (3.3 / 4095.0);
  float phValue = 7.0 + ((2.5 - voltage) / 2.5) * 3.5;
  if (phValue < 0)  phValue = 0;
  if (phValue > 14) phValue = 14;
  return phValue;
}

// ---------- Arduino setup ----------
void setup() {
  Serial.begin(115200);
  
  // Initialize I2C (default pins: SDA=21, SCL=22 on ESP32) [web:34]
  Wire.begin();
  setupLCD();
  
  dht.begin();
  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);

  setupWiFi();  // Now shows IP on LCD too

  client.setServer(mqtt_server, mqtt_port);
}

// ---------- Main loop ----------
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;

    float temp = readTemperature();
    float hum  = readHumidity();
    float tds  = readTDS();
    float ph   = readPH();

    if (temp < -500 || hum < 0) {
      Serial.println("DHT error, skipping publish");
      return;
    }

    // Build JSON payload
    char payload[200];
    snprintf(payload, sizeof(payload),
             "{\"temperature\":%.2f,\"humidity\":%.2f,\"tds\":%.2f,\"ph\":%.2f}",
             temp, hum, tds, ph);

    Serial.print("Publishing: ");
    Serial.println(payload);

    if (client.publish(mqtt_topic, payload)) {
      Serial.println("Publish OK");
    } else {
      Serial.println("Publish FAILED");
    }
  }
}
