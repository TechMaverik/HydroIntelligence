from flask import Flask, render_template
from datetime import datetime
import socket
import json
import threading

import paho.mqtt.client as mqtt

app = Flask(__name__)

# ---------- MQTT CONFIG ----------
MQTT_BROKER = "broker.mqtt.cool"  # change if needed
MQTT_PORT = 1883
MQTT_TOPIC = "/sensordata"  # change if needed

# ---------- GLOBAL SENSOR DATA ----------
sensor_data = {
    "temperature": 0,
    "humidity": 0,
    "tds": 0,
    "ph": 0,
    "relay": "OFF",
    "ip": "0.0.0.0",  # ESP32 / device IP
}


# ---------- MQTT CALLBACKS ----------
def on_connect(client, userdata, flags, rc):
    # print("Connected to MQTT broker with code:", rc)
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    global sensor_data
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)

        # Read values from MQTT JSON
        sensor_data["temperature"] = data.get("temperature", 0)
        sensor_data["humidity"] = data.get("humidity", 0)
        sensor_data["tds"] = data.get("tds", 0)
        sensor_data["ph"] = data.get("ph", 0)
        sensor_data["relay"] = data.get("relay", "OFF")

        # ✅ DEVICE IP FROM MQTT PAYLOAD
        sensor_data["ip"] = data.get("ip", "0.0.0.0")

        # print("MQTT Data Received:", sensor_data)

    except Exception as e:
        print("MQTT message error:", e)


# ---------- MQTT THREAD ----------
def mqtt_loop():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()


# Start MQTT in background thread
threading.Thread(target=mqtt_loop, daemon=True).start()


# ---------- FLASK ROUTE ----------
@app.route("/")
def dashboard():
    return render_template(
        "dashboard.html",
        temperature=sensor_data["temperature"],
        humidity=sensor_data["humidity"],
        tds=sensor_data["tds"],
        ph=sensor_data["ph"],
        relay=sensor_data["relay"],
        device_ip=sensor_data["ip"],  # ESP32 IP
        server_ip=socket.gethostbyname(socket.gethostname()),
        current_time=datetime.now().strftime("%d-%m-%Y %H:%M:%S"),
    )


# ---------- MAIN ----------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=2025, debug=True)
