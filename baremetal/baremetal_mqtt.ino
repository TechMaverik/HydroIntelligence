#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>                    // I2C support
#include <LiquidCrystal_I2C.h>       // I2C LCD library
#include <RTClib.h>                  // RTC library

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

// ---------- LCD (I2C) ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Address 0x27, 16x2 display

// ---------- RTC ----------
RTC_DS3231 rtc;

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
  lcd.print("Getting time..");
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

// ---------- Helper: init RTC ----------
void setupRTC() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.clear();
    lcd.print("RTC ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Check wiring");
    delay(3000);
  }
  
  // Show current time on LCD when setting RTC
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting time");
    
    DateTime compileTime(F(__DATE__), F(__TIME__));
    rtc.adjust(compileTime);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RTC: Set Time");
    lcd.setCursor(0, 1);
    
    char timeStr[17];
    sprintf(timeStr, "%s %s", compileTime.timestamp().c_str(), compileTime.timestamp().c_str());
    lcd.print(timeStr);
    delay(3000);
  } else {
    Serial.println("RTC OK - Current time set");
  }
  
  // Display current time briefly after RTC setup
  DateTime now = rtc.now();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time synced:");
  lcd.setCursor(0, 1);
  char currentTime[9];
  sprintf(currentTime, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
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
void updateRelay(DateTime now) {
  int currentMinute = now.hour() * 60 + now.minute();
  
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
void displayTimeIP(DateTime now) {
  lcd.clear();
  
  // Line 1: Time HH:MM:SS
  lcd.setCursor(0, 0);
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  lcd.print(timeStr);
  
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
void displayRelayStatus(DateTime now) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Relay:");
  lcd.print(relayState ? "ON " : "OFF");
  
  int currentMinute = now.hour() * 60 + now.minute();
  int cyclePosition = (currentMinute - relayMinuteStart) % 30;
  int timeLeft = 15 - cyclePosition;
  if (timeLeft < 0) timeLeft += 30;
  
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
  
  setupRTC();
  setupWiFi();
  
  lcd.clear();
  lcd.print("All systems OK!");
  lcd.setCursor(0, 1);
  lcd.print("Relay ready");
  delay(1500);
  
  client.setServer(mqtt_server, mqtt_port);
}

// ---------- Main loop ----------
void loop() {
  // MQTT maintenance
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  DateTime rtcTime = rtc.now();
  
  // Update relay control every loop (RTC-based, precise timing)
  updateRelay(rtcTime);

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
      case 0: displayTimeIP(rtcTime); break;
      case 1: displaySensors(temp, hum, tds, ph); break;
      case 2: displayRelayStatus(rtcTime); break;
    }

    // MQTT payload with relay status included
    char payload[400];
    snprintf(payload, sizeof(payload),
             "{\"time\":\"%04d-%02d-%02dT%02d:%02d:%02d\","
             "\"temp\":%.1f,\"hum\":%.1f,\"tds\":%.0f,\"ph\":%.1f,"
             "\"relay\":%s,\"rssi\":%d,\"ip\":\"%s\"}",
             rtcTime.year(), rtcTime.month(), rtcTime.day(),
             rtcTime.hour(), rtcTime.minute(), rtcTime.second(),
             temp, hum, tds, ph,
             relayState ? "ON" : "OFF",
             WiFi.RSSI(), WiFi.localIP().toString().c_str());

    Serial.println(payload);
    
    client.publish(mqtt_topic, payload);
  }
}
