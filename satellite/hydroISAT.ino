#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

// ==== OLED Setup ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const char* ssid = "DPSSRW-M30";
const char* password = "Dpsob@19216";
const char* mqtt_server = "broker.mqtt.cool";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";
const char* mqtt_topic = "/sensordata";
const char* client_id = "D1MiniDisplay";

WiFiClient espClient;
PubSubClient client(espClient);

// ==== Data Variables ====
String temp = "--";
String hum  = "--";
String ph   = "--";
String tds  = "--";
String relay1 = "--";
String relay2 = "--";

// ==== Helper: Show message on OLED ====
void showMessage(String msg, int delayTime) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(msg);
  display.display();
  delay(delayTime);
}

// ==== Get WiFi strength as string ====
String getWifiStrength() {
  long rssi = WiFi.RSSI();
  if (rssi > -30) return "EXCELLENT";
  if (rssi > -67) return "GOOD";
  if (rssi > -70) return "OK";
  if (rssi > -80) return "WEAK";
  return "POOR";
}

// ==== MQTT Callback with DEBUG ====
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("MQTT Message received on topic: " + String(topic));
  
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.println("Raw payload: " + message);
  
  StaticJsonDocument<400> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  temp = String(doc["temperature"] | -999.0, 1);
  hum  = String(doc["humidity"] | -999.0, 1);
  ph   = String(doc["ph"] | -999.0, 2);
  tds  = String(doc["tds"] | -999.0, 0);
  relay1 = doc["relay1"] | "ERR";
  relay2 = doc["relay2"] | "ERR";
  
  Serial.println("Parsed - T:" + temp + " H:" + hum + " pH:" + ph + " TDS:" + tds);
}

// ==== Connect to WiFi with status display ====
void setup_wifi() {
  showMessage("Connecting WiFi...", 0);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected! IP: " + WiFi.localIP().toString());
    showMessage("WiFi: OK", 1500);
  } else {
    Serial.println("WiFi failed!");
    showMessage("WiFi FAILED", 3000);
    ESP.restart();
  }
}

// ==== Reconnect MQTT if disconnected ====
void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connect attempt... State: ");
    Serial.println(client.state());
    
    showMessage("Cloud Connect...", 1000);
    
    if (client.connect(client_id)) {
      Serial.println("Cloud Connected!");
      showMessage("Cloud Status: OK", 1000);
      
      if (client.subscribe(mqtt_topic, 1)) {
        Serial.println("Subscribed to: " + String(mqtt_topic));
      } else {
        Serial.println("Subscribe FAILED!");
      }
    } else {
      Serial.print("MQTT failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== DPS LAB Display Starting ===");
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    for(;;);
  }
  display.clearDisplay();

  // Startup messages - FIXED
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 0);
  display.println("AI Robotics & DnT LAB");
  display.setCursor(2, 16);
  display.println("DPS Indirapuram");
  display.setCursor(2, 32);
  display.println("Presented to:");
  display.setCursor(2, 48);
  display.println("Mr Girish Sachdev");
  display.display();
  delay(4000);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  Serial.println("Setup complete. Monitoring MQTT...");
}

// ==== Loop ====
void loop() {
  if (WiFi.status() != WL_CONNECTED) {  // FIXED: Corrected logic
    Serial.println("WiFi lost! Restarting...");
    ESP.restart();
  }
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Update display every 2 seconds - FIXED LINE 197 WITH WIFI STRENGTH
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    
    int y = 0;
    display.setCursor(0, y); y += 10; display.println("HydroFarmIntelligence"); 
    display.setCursor(0, y); y += 10; display.print("Temp: "); display.println(temp);
    display.setCursor(0, y); y += 10; display.print("Hum:  "); display.println(hum); 
    // display.setCursor(0, y); y += 10; display.print("pH:   "); display.println(ph);
    display.setCursor(0, y); y += 10; display.print("TDS:  "); display.print(tds); display.print("   pH:   "); display.println(ph);
    display.setCursor(0, y); y += 10; display.print("PUMP:  "); display.println(relay1);
    
    // LINE 197: WIFI STRENGTH + STATUS
    display.setCursor(0, y); 
    display.print("WiFi: "); 
    display.print(getWifiStrength()); 
    display.print(" "); 
    display.print(WiFi.RSSI()); 
    display.println("dBm");
    
    display.display();
  }
}
