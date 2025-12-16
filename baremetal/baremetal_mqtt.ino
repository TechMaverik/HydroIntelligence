#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

// ---------- Pins ----------
#define DHT_PIN   4
#define DHT_TYPE  DHT11
#define TDS_PIN   34
#define PH_PIN    35
#define RELAY_PIN 2
#define RELAY_PIN2 5

// ---------- WiFi ----------
const char* ssid     = "Ai Lab";
const char* password = "Welc0me@123";

// ---------- MQTT ----------
const char* mqtt_server = "broker.mqtt.cool";
const int   mqtt_port   = 1883;
const char* mqtt_topic = "/sensor_data_stream";

// ---------- NTP ----------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);

// ---------- LCD ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Globals ----------
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastPublish = 0;
const unsigned long publishInterval = 5000;

unsigned long lastDisplaySwitch = 0;
const unsigned long displayInterval = 3000;
int displayMode = 0;

bool relayState = false;

// ---------- LCD Init ----------
void setupLCD() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AI & ROBOTICS");
  lcd.setCursor(0, 1);
  lcd.print("D&T LAB DPSI");
  delay(3000);
  lcd.clear();
}

// ---------- WiFi ----------
void setupWiFi() {
  WiFi.begin(ssid, password);
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }
  lcd.clear();
}

// ---------- NTP ----------
void setupNTP() {
  timeClient.begin();
  timeClient.update();
}

// ---------- MQTT ----------
void reconnectMQTT() {
  while (!client.connected()) {
    String clientId = "hydrointel-";
    clientId += String(random(0xffff), HEX);
    client.connect(clientId.c_str());
    delay(1000);
  }
}

// ---------- Relay Logic (15-minute toggle) ----------
void updateRelay() {
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);

  int totalMinutes = ptm->tm_hour * 60 + ptm->tm_min;
  int block15 = totalMinutes / 15;

  bool shouldBeOn = (block15 % 2 == 0);

  if (relayState != shouldBeOn) {
    relayState = shouldBeOn;
    
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    digitalWrite(RELAY_PIN2, relayState ? HIGH : LOW);
  }
}

// ---------- Displays ----------
void displayTimeIP() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(timeClient.getFormattedTime());
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
}

void displaySensors(float t, float h, float tds, float ph) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t, 1);
  lcd.print(" H:");
  lcd.print(h, 0);

  lcd.setCursor(0, 1);
  lcd.print("pH:");
  lcd.print(ph, 1);
  lcd.print(" T:");
  lcd.print((int)tds);
}

void displayRelayStatus() {
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime((time_t *)&epochTime);

  int remaining = 15 - (ptm->tm_min % 15);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pump:");
  lcd.print(relayState ? "ON " : "OFF");

  lcd.setCursor(0, 1);
  lcd.print("Next in ");
  lcd.print(remaining);
  lcd.print("m");
}

// ---------- Sensors ----------
float readTemperature() {
  float v = dht.readTemperature();
  return isnan(v) ? -1000 : v;
}

float readHumidity() {
  float v = dht.readHumidity();
  return isnan(v) ? -1 : v;
}

float readTDS() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(TDS_PIN);
    delay(10);
  }
  float v = (sum / 10.0) * (3.3 / 4095.0);
  return (133.42 * v * v * v - 255.86 * v * v + 857.39 * v) * 0.5;
}

float readPH() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(PH_PIN);
    delay(10);
  }
  float v = (sum / 10.0) * (3.3 / 4095.0);
  float ph = 7.0 + ((2.5 - v) / 2.5) * 3.5;
  return constrain(ph, 0, 14);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN2, OUTPUT);
  digitalWrite(RELAY_PIN2, LOW);

  Wire.begin();
  setupLCD();
  dht.begin();

  setupWiFi();
  setupNTP();

  client.setServer(mqtt_server, mqtt_port);
}

// ---------- Loop ----------
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  updateRelay();

  unsigned long now = millis();

  if (now - lastDisplaySwitch > displayInterval) {
    lastDisplaySwitch = now;
    displayMode = (displayMode + 1) % 3;
  }

  if (now - lastPublish > publishInterval) {
    lastPublish = now;

    float t = readTemperature();
    float h = readHumidity();
    float tds = readTDS();
    float ph = readPH();

    if (displayMode == 0) displayTimeIP();
    if (displayMode == 1) displaySensors(t, h, tds, ph);
    if (displayMode == 2) displayRelayStatus();

    IPAddress ip = WiFi.localIP();

    char payload[300];
    snprintf(payload, sizeof(payload),
      "{\"temp\":%.1f,\"hum\":%.1f,\"tds\":%.0f,\"ph\":%.1f,"
      "\"relay\":\"%s\",\"ip\":\"%d.%d.%d.%d\"}",
      t, h, tds, ph,
      relayState ? "ON" : "OFF",
      ip[0], ip[1], ip[2], ip[3]
    );

    client.publish(mqtt_topic, payload);
    Serial.println(payload);
  }
}
