#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>                    // I2C support
#include <LiquidCrystal_I2C.h>       // I2C LCD library
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

// ---------- Pins ----------
#define DHT_PIN   4
#define DHT_TYPE  DHT11
#define TDS_PIN   34
#define PH_PIN    35
#define RELAY_PIN 2    // Relay control pin

// ---------- WiFi ----------
const char* ssid     = "Ai Lab";
const char* password = "Welc0me@123";

// ---------- MQTT ----------
const char* mqtt_server = "broker.mqtt.cool";
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "/sensor_data_stream";

// ---------- NTP ----------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000); // IST timezone offset

// ---------- LCD (I2C) ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16x2 display

// ---------- Globals ----------
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000;  // ms
int displayMode = 0;  // 0=Time+IP, 1=Sensors, 2=Relay Status
unsigned long lastDisplaySwitch = 0;
const unsigned long displayInterval = 3000;  // Switch every 3 sec

bool relayState = false;  // Track relay state
int relayMinuteStart = -1; // Starting minute of current 30-min cycle

// ---------- Helper: init LCD ----------
void setupLCD() {
  lcd.init();
  lcd.backlight();
  
  // Display "AI & ROBOTICS" on first line
  lcd.setCursor(0, 0);
  lcd.print("AI & ROBOTICS");
  
  // Display "D&T LAB DPSI" on second line
  lcd.setCursor(0, 1);
  lcd.print("D&T LAB DPSI");
  
  delay(3000);
  
  // Show current time during upload/initialization
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Getting NTP time..");
  delay(2000);
  lcd.clear();
}

// ---------- Helper: connect WiFi ----------
void setupWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  lcd.setCursor(0, 0);
  lcd.print("WiFi:.....");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    lcd.setCursor(12, 0);
    lcd.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ---------- Helper: init NTP time ----------
void setupNTP() {
  timeClient.begin();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Syncing NTP...");
  
  // Force initial sync
  timeClient.update();
  
  // Display sync status and current time
  lcd.setCursor(0, 1);
  if (timeClient.isTimeSet()) {
    lcd.print("NTP OK");
    Serial.println("NTP synchronized");
  } else {
    lcd.print("NTP FAIL");
    Serial.println("NTP sync failed");
  }
  
  delay(2000);
  
  // Display current time briefly after NTP setup
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time synced:");
  lcd.setCursor(0, 1);
  String currentTime = timeClient.getFormattedTime();
  lcd.print(currentTime);
  delay(2000);
  lcd.clear();
}

// ---------- Helper: connect MQTT ----------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT...");
    String clientId = "hydrointel-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 5s");
      delay(5000);
    }
  }
}

// ---------- Relay control logic ----------
void updateRelay() {
  timeClient.update(); // Keep NTP updated
  unsigned long epochTime = timeClient.getEpochTime();
  
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int currentMinute = ptm->tm_hour * 60 + ptm->tm_min;
  
  // Check if new 30-minute cycle started
  if (currentMinute / 30 != relayMinuteStart / 30) {
    relayMinuteStart = currentMinute;
    relayState = !relayState;  // Toggle state every 30 minutes
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    
    Serial.print("Relay toggled to: ");
    Serial.println(relayState ? "ON" : "OFF");
    Serial.print("Cycle start minute: ");
    Serial.println(relayMinuteStart);
  }
  
  // Determine if relay should be ON (first 15 min) or OFF (next 15 min)
  int cyclePosition = (currentMinute - relayMinuteStart) % 30;
  bool shouldBeOn = (cyclePosition < 15);
  
  if (relayState != shouldBeOn) {
    relayState = shouldBeOn;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    Serial.print("Relay corrected to: ");
    Serial.println(relayState ? "ON" : "OFF");
  }
}

// ---------- Display Mode 0: Time + IP ----------
void displayTimeIP() {
  lcd.clear();
  timeClient.update();
  
  // Line 1: Time HH:MM:SS
  lcd.setCursor(0, 0);
  lcd.print(timeClient.getFormattedTime().substring(0, 8));
  
  // Line 2: IP address
  lcd.setCursor(0, 1);
  lcd.print("IP:");
  lcd.print(WiFi.localIP().toString());
}

// ---------- Display Mode 1: All Sensors ----------
void displaySensors(float temp, float hum, float tds, float ph) {
  lcd.clear();
  
  // Line 1: T + H
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C H:");
  lcd.print(hum, 0);
  lcd.print("%");
  
  // Line 2: pH + TDS
  lcd.setCursor(0, 1);
  lcd.print("pH:");
  lcd.print(ph, 1);
  lcd.print(" TDS:");
  lcd.print((int)tds);
  lcd.print("ppm");
}

// ---------- Display Mode 2: Relay Status ----------
void displayRelayStatus() {
  lcd.clear();
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  
  struct tm *ptm = gmtime((time_t *)&epochTime);
  int currentMinute = ptm->tm_hour * 60 + ptm->tm_min;
  int cyclePosition = (currentMinute - relayMinuteStart) % 30;
  
  lcd.setCursor(0, 0);
  lcd.print("Relay:");
  lcd.print(relayState ? "ON " : "OFF");
  
  lcd.setCursor(0, 1);
  lcd.print("Cycle:");
  lcd.print(cyclePosition);
  lcd.print("m/30 ");
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
  float avgRaw = raw / (float)samples;
  float voltage = avgRaw * (3.3 / 4095.0);
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
  float avgRaw = raw / (float)samples;
  float voltage = avgRaw * (3.3 / 4095.0);
  float phValue = 7.0 + ((2.5 - voltage) / 2.5) * 3.5;
  if (phValue < 0) phValue = 0;
  if (phValue > 14) phValue = 14;
  return phValue;
}

// ---------- Arduino setup ----------
void setup() {
  Serial.begin(115200);
  
  // Initialize relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Start with relay OFF
  
  Wire.begin();  // SDA=21, SCL=22
  setupLCD();
  
  dht.begin();
  pinMode(TDS_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  
  setupWiFi();
  setupNTP();
  
  lcd.clear();
  lcd.print("All systems OK!");
  lcd.setCursor(0, 1);
  lcd.print("NTP+Relay ready");
  delay(1500);
  
  client.setServer(mqtt_server, mqtt_port);
}

// ---------- Main loop ----------
void loop() {
  // NTP update every loop for accurate timing
  timeClient.update();
  
  // MQTT maintenance
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Update relay control every loop (NTP-based, precise timing)
  updateRelay();

  // Update display every 3 seconds
  unsigned long now = millis();
  if (now - lastDisplaySwitch >= displayInterval) {
    lastDisplaySwitch = now;
    displayMode = (displayMode + 1) % 3;  // Cycle 0,1,2
  }

  // Data publish every 5 seconds
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;
    
    float temp = readTemperature();
    float hum = readHumidity();
    float tds = readTDS();
    float ph = readPH();

    if (temp < -500 || hum < 0) {
      Serial.println("DHT error");
      return;
    }

    // Update display based on current mode
    switch(displayMode) {
      case 0: displayTimeIP(); break;
      case 1: displaySensors(temp, hum, tds, ph); break;
      case 2: displayRelayStatus(); break;
    }

    // MQTT payload with NTP time and relay status
    unsigned long epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime);
    char payload[400];
    snprintf(payload, sizeof(payload),
             "{\"time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
             "\"temp\":%.1f,\"hum\":%.1f,\"tds\":%.0f,\"ph\":%.1f,"
             "\"relay\":%s,\"rssi\":%d,\"ip\":\"%s\"}",
             ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
             ptm->tm_hour, ptm->tm_min, ptm->tm_sec,
             temp, hum, tds, ph,
             relayState ? "ON" : "OFF",
             WiFi.RSSI(), WiFi.localIP().toString().c_str());

    Serial.println(payload);
    
    client.publish(mqtt_topic, payload);
  }
}
