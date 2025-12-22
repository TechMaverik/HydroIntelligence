from flask import Flask, render_template
from datetime import datetime
import socket
import json
import threading
import csv
import os

import paho.mqtt.client as mqtt

app = Flask(__name__)

# ---------- MQTT CONFIG ----------
MQTT_BROKER = "broker.mqtt.cool"  # change if needed
MQTT_PORT = 1883
MQTT_TOPIC = "/sensordata"  # change if needed

# ---------- CSV CONFIG ----------
CSV_FILE = "hydrointelligencelog.csv"

# create file with header if not exists
if not os.path.exists(CSV_FILE):
    with open(CSV_FILE, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "date",
                "time",
                "temperature",
                "humidity",
                "tds",
                "ph",
                "relay",
                "device_ip",
                "server_ip",
            ]
        )

# ---------- GLOBAL SENSOR DATA ----------
sensor_data = {
    "temperature": 0,
    "humidity": 0,
    "tds": 0,
    "ph": 0,
    "relay": "OFF",
    "ip": "0.0.0.0",  # ESP32 / device IP
}

# keep last logged values to detect change
last_logged = sensor_data.copy()


def log_to_csv_if_changed():
    """
    Append a row to CSV only when any sensor_data field changed
    compared to last_logged.
    """
    global last_logged

    # check if anything changed
    if sensor_data == last_logged:
        return  # no change, skip logging

    now = datetime.now()
    date_str = now.strftime("%d-%m-%Y")
    time_str = now.strftime("%H:%M:%S")

    server_ip = socket.gethostbyname(socket.gethostname())

    row = [
        date_str,
        time_str,
        sensor_data["temperature"],
        sensor_data["humidity"],
        sensor_data["tds"],
        sensor_data["ph"],
        sensor_data["relay"],
        sensor_data["ip"],
        server_ip,
    ]

    try:
        with open(CSV_FILE, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)
        # update last_logged after successful write
        last_logged = sensor_data.copy()
    except Exception as e:
        print("CSV log error:", e)


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

        # DEVICE IP FROM MQTT PAYLOAD
        sensor_data["ip"] = data.get("ip", "0.0.0.0")

        # log to CSV if there is any change
        log_to_csv_if_changed()

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
    temperature = sensor_data["temperature"]
    humidity = sensor_data["humidity"]
    tds = sensor_data["tds"]
    ph = sensor_data["ph"]
    relay = sensor_data["relay"]
    device_ip = sensor_data["ip"]  # ESP32 IP
    server_ip = socket.gethostbyname(socket.gethostname())
    current_time = datetime.now().strftime("%d-%m-%Y %H:%M:%S")

    return render_template(
        "dashboard.html",
        temperature=temperature,
        humidity=humidity,
        tds=tds,
        ph=ph,
        relay=relay,
        device_ip=device_ip,  # ESP32 IP
        server_ip=server_ip,
        current_time=current_time,
    )


# ---------- MAIN ----------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=2025, debug=True)
