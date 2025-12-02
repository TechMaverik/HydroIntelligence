#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <DHT.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define DHT_PIN 4
#define DHT_TYPE DHT11
#define TDS_PIN 34
#define PH_PIN 35

#define WIFI_SSID "Ai Lab"
#define WIFI_PASSWORD "Welc0me@123"
#define API_KEY "AIzaSyAEPMG1VYq3kykLYNCYAkds3wLNb-n5HSM"
#define DATABASE_URL "https://hydrointelligence-37338-default-rtdb.asia-southeast1.firebasedatabase.app/"

DHT dht(DHT_PIN, DHT_TYPE);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

float prevTemp = -999, prevHum = -999, prevTDS = -999, prevPH = -999;
const float CHANGE_THRESHOLD = 0.1;
unsigned long lastReading = 0;
const unsigned long READ_INTERVAL = 2000;

// Function prototypes to avoid "not declared" errors
float readTDS(float temp);
float readPH();

void setup() {
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWiFi Connected!");

  // API key auth only - NO signup or callback needed
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Firebase Ready!");
}



void loop() {
  if (Firebase.ready() && (millis() - lastReading > READ_INTERVAL)) {
    lastReading = millis();

    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    float tds = readTDS(temp);
    float ph = readPH();

    if (isnan(temp) || isnan(hum)) {
      Serial.println("Failed to read DHT!");
      return;
    }

    Serial.printf("T:%.1f°C H:%.1f%% TDS:%.0fppm pH:%.2f\n", temp, hum, tds, ph);

    bool changed = false;
    if (abs(temp - prevTemp) > CHANGE_THRESHOLD) changed = true;
    if (abs(hum - prevHum) > CHANGE_THRESHOLD) changed = true;
    if (abs(tds - prevTDS) > 1.0) changed = true;
    if (abs(ph - prevPH) > 0.05) changed = true;

    if (changed) {
      prevTemp = temp;
      prevHum = hum;
      prevTDS = tds;
      prevPH = ph;

      sendToFirebase(temp, hum, tds, ph);
      Serial.println("Data sent on change!");
    }
  }
}

void sendToFirebase(float temp, float hum, float tds, float ph) {
  String path = "/sensors/latest";

  if (!Firebase.RTDB.setFloat(&fbdo, path + "/temperature", temp))
    Serial.println(fbdo.errorReason());
  if (!Firebase.RTDB.setFloat(&fbdo, path + "/humidity", hum))
    Serial.println(fbdo.errorReason());
  if (!Firebase.RTDB.setFloat(&fbdo, path + "/tds", tds))
    Serial.println(fbdo.errorReason());
  if (!Firebase.RTDB.setFloat(&fbdo, path + "/ph", ph))
    Serial.println(fbdo.errorReason());
  if (!Firebase.RTDB.setTimestamp(&fbdo, path + "/timestamp"))
    Serial.println(fbdo.errorReason());
}

float readTDS(float temp) {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(TDS_PIN);
    delay(10);
  }
  float voltage = (sum / 10) * (3.3 / 4095.0);
  float compensation = (1.0 + 0.02 * (temp - 25.0));
  float tds = (133.42 * voltage * voltage * voltage -
               255.86 * voltage * voltage + 857.39 * voltage) * 0.7 * compensation;
  return tds < 0 ? 0 : tds;
}

float readPH() {
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(PH_PIN);
    delay(10);
  }
  float voltage = (sum / 10) * (3.3 / 4095.0);
  float phValue = 7.0 + ((2.5 - voltage) / 2.5) * 3.5;
  return constrain(phValue, 0, 14);
}
