# main.py
import os
import threading
import json
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import paho.mqtt.client as mqtt

# ---- KONFIGURÁCIÓ ----
DEFAULT_BROKER = "mosquitto"
MQTT_HOST = os.getenv("MQTT_HOST", DEFAULT_BROKER)
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

MQTT_CMD_TOPIC = os.getenv("MQTT_CMD_TOPIC", "robot/cmd")          # String parancsok
MQTT_VEL_TOPIC = "robot/cmd/vel"                                   # ebesség parancsok
MQTT_STATE_TOPIC = os.getenv("MQTT_STATE_TOPIC", "robot/state")    # Telemetria
MQTT_PING_TOPIC = "dogzilla/control/ping"                          # Látencia mérés

# ---- PARANCSLISTA (Trükkök és diszkrét mozgás) ----
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
last_state: dict = {"status": "unknown", "message": "Waiting for robot data..."}
mqtt_lock = threading.Lock()

# ---- ADATMODELLEK ----

class CommandIn(BaseModel):
    cmd: str

# ÚJ: Modell a folyamatos sebességvezérléshez
class VelocityIn(BaseModel):
    vx: float       # Előre/Hátra (-1.0 ... 1.0)
    wz: float       # Forgás (-1.0 ... 1.0)
    vy: float = 0.0 # Oldalazás (opcionális)


# ---- MQTT CALLBACK-EK ----

def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] Csatlakozva ({MQTT_HOST}:{MQTT_PORT})")
    client.subscribe(MQTT_STATE_TOPIC)

def on_message(client, userdata, msg: mqtt.MQTTMessage):
    global last_state
    try:
        payload_str = msg.payload.decode("utf-8")
        if msg.topic == MQTT_STATE_TOPIC:
            try:
                state_data = json.loads(payload_str)
                last_state = state_data
            except json.JSONDecodeError:
                last_state = {"raw_message": payload_str}
    except Exception as e:
        print(f"[HIBA] Üzenet hiba: {e}")


# ---- FASTAPI ÉLETCIKLUS ----

@app.on_event("startup")
def startup_event():
    global mqtt_client
    print(f"[Startup] Csatlakozás MQTT-hez...")
    mqtt_client = mqtt.Client(client_id="fastapi_backend")
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"[KRITIKUS HIBA] MQTT hiba: {e}")

@app.on_event("shutdown")
def shutdown_event():
    global mqtt_client
    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


# ---- ENDPOINTOK ----

@app.post("/cmd")
async def send_command(cmd_in: CommandIn):
    """ String parancs (pl. 'sit', 'forward') küldése """
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT hiba"}

    command = cmd_in.cmd.lower().strip()
    if command not in VALID_COMMANDS:
        return {"status": "error", "message": "Érvénytelen parancs"}

    with mqtt_lock:
        mqtt_client.publish(MQTT_CMD_TOPIC, command)

    return {"status": "ok", "sent": command}


@app.post("/cmd/vel")
async def send_velocity(vel: VelocityIn):
    """ 
    ÚJ: Folyamatos sebességvezérlés (Joystick).
    JSON Body: {"vx": 1.0, "wz": 0.5}
    """
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT hiba"}

    # JSON csomag összeállítása a Bridge Node számára
    payload = {
        "vx": vel.vx,
        "vy": vel.vy,
        "wz": vel.wz
    }
    
    with mqtt_lock:
        # A robot/cmd/vel topikra küldjük JSON-ként!
        mqtt_client.publish(MQTT_VEL_TOPIC, json.dumps(payload))

    return {"status": "ok", "sent": payload}


@app.get("/state")
async def get_state():
    """ Telemetria adatok lekérdezése """
    return {"status": "ok", "data": last_state}


@app.post("/ping")
async def send_ping():
    """ Látencia mérés """
    if mqtt_client is None: return {"error": "MQTT hiba"}
    with mqtt_lock:
        mqtt_client.publish(MQTT_PING_TOPIC, "PING_TEST")
    return {"status": "ping_sent"}
