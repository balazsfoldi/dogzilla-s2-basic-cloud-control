# main.py
import os
import threading
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import paho.mqtt.client as mqtt

# ---- Konfiguráció ----
# FIGYELEM: A teszthez állítsd be a nyilvános brókert, ha nincs sajátod!
MQTT_HOST = os.getenv("MQTT_HOST", "broker.hivemq.com") 
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_CMD_TOPIC = os.getenv("MQTT_CMD_TOPIC", "robot/cmd")
MQTT_STATE_TOPIC = os.getenv("MQTT_STATE_TOPIC", "robot/state")

# Külön téma a méréshez (hogy illeszkedjen a robot scriptjéhez)
MQTT_PING_TOPIC = "dogzilla/control/ping"

VALID_COMMANDS = {
    "forward", "backward", "left", "right", 
    "turn_left", "turn_right", "stop",
    "lie_down", "stand_up", "crawl", "turn_around",
    "mark_time", "squat", "turn_roll", "turn_pitch",
    "turn_yaw", "3_axis", "pee", "sit",
    "wave_hand", "stretch", "wave_body", "swing",
    "pray", "seek", "handshake", "push_up"
}

app = FastAPI(title="Dogzilla MQTT Controller")

mqtt_client: Optional[mqtt.Client] = None
last_state: Optional[str] = "Ismeretlen"
mqtt_lock = threading.Lock()

class CommandIn(BaseModel):
    cmd: str

# ---- MQTT callback-ek ----
def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] Csatlakozva (kód={reason_code})")
    client.subscribe(MQTT_STATE_TOPIC)

def on_message(client, userdata, msg: mqtt.MQTTMessage):
    global last_state
    payload = msg.payload.decode("utf-8")
    if msg.topic == MQTT_STATE_TOPIC:
        last_state = payload

# ---- FastAPI lifecycle ----
@app.on_event("startup")
def startup_event():
    global mqtt_client
    print(f"[Startup] Csatlakozás MQTT-hez: {MQTT_HOST}:{MQTT_PORT} ...")
    # MQTTv5 helyett v3.1.1-et használok a kompatibilitás miatt, de maradhat v5 is
    mqtt_client = mqtt.Client(client_id="fastapi_controller") 
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        mqtt_client.loop_start()
        print("[Startup] MQTT kliens fut.")
    except Exception as e:
        print(f"[HIBA] Nem sikerült csatlakozni: {e}")

@app.on_event("shutdown")
def shutdown_event():
    global mqtt_client
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

# ---- REST endpointok ----

@app.post("/cmd")
async def send_command(cmd_in: CommandIn):
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT nincs csatlakoztatva"}

    command = cmd_in.cmd.lower().strip()
    if command not in VALID_COMMANDS:
        return {"status": "error", "message": "Érvénytelen parancs!"}

    with mqtt_lock:
        mqtt_client.publish(MQTT_CMD_TOPIC, command)

    return {"status": "ok", "sent": command}

# --- ÚJ ENDPOINT A MÉRÉSHEZ ---
@app.post("/ping")
async def send_ping():
    """
    Ez az endpoint csak a latency méréshez kell.
    Kiküldi a PING_TEST jelet a dogzilla/control/ping témára.
    """
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT hiba"}

    with mqtt_lock:
        # Fontos: Ugyanoda küldjük, ahova a robot script figyel!
        mqtt_client.publish(MQTT_PING_TOPIC, "PING_TEST")
    
    return {"status": "ping_sent"}
