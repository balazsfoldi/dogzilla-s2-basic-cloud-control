import json
import random
import time

import paho.mqtt.client as mqtt

BROKER_HOST = "mosquitto"
BROKER_PORT = 1883

state = {
    "status": "idle",
    "battery": 100,
    "position": {
        "x": 0,
        "y": 0,
        "theta": 0
    },
    "last_command": None
}

client = mqtt.Client()


def on_connect(client, userdata, flags, rc):
    print(f"[MQTT] Connected: {rc}")

    client.subscribe("robot/cmd")
    client.subscribe("dogzilla/control/ping")


def on_message(client, userdata, msg):
    global state

    payload = msg.payload.decode()

    print(f"[MQTT] {msg.topic}: {payload}")

    if msg.topic == "robot/cmd":

        state["last_command"] = payload

        if payload == "forward":
            state["position"]["x"] += 1
            state["status"] = "moving"

        elif payload == "backward":
            state["position"]["x"] -= 1
            state["status"] = "moving"

        elif payload == "left":
            state["position"]["y"] -= 1
            state["status"] = "moving"

        elif payload == "right":
            state["position"]["y"] += 1
            state["status"] = "moving"

        elif payload == "stop":
            state["status"] = "idle"

    elif msg.topic == "dogzilla/control/ping":

        client.publish(
            "robot/state",
            json.dumps({
                "type": "pong",
                "timestamp": time.time()
            })
        )


client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_HOST, BROKER_PORT, 60)
client.loop_start()

while True:

    state["battery"] -= random.uniform(0, 0.02)

    if state["battery"] < 0:
        state["battery"] = 0

    client.publish(
        "robot/state",
        json.dumps(state)
    )

    print("[STATE]", state)

    time.sleep(2)