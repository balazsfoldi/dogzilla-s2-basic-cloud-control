# main.py
import os
import threading
import json
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import paho.mqtt.client as mqtt

# ---- KONFIGURÁCIÓ ----
# A Kubernetesben a 'mosquitto' service nevet használjuk, helyi tesztnél 'localhost' vagy 'broker.hivemq.com'
DEFAULT_BROKER = "mosquitto" 
# Ha a környezeti változó nincs beállítva, akkor a default-ot használja
MQTT_HOST = os.getenv("MQTT_HOST", DEFAULT_BROKER)
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

MQTT_CMD_TOPIC = os.getenv("MQTT_CMD_TOPIC", "robot/cmd")
MQTT_STATE_TOPIC = os.getenv("MQTT_STATE_TOPIC", "robot/state")
MQTT_PING_TOPIC = "dogzilla/control/ping" # Külön téma a méréshez

# ---- FRISSÍTETT PARANCSLISTA (DOGZILLA S2) ----
VALID_COMMANDS = {
    # 1. Alapvető mozgás
    "forward", "backward", "left", "right", 
    "turn_left", "turn_right", "stop",
    
    # 2. Trükkök (Action kódok)
    "lie_down", "stand_up", "crawl", "turn_around",
    "mark_time", "squat", "turn_roll", "turn_pitch",
    "turn_yaw", "3_axis", "pee", "sit",
    "wave_hand", "stretch", "wave_body", "swing",
    "pray", "seek", "handshake", "push_up"
}

app = FastAPI(title="Dogzilla MQTT Controller")

mqtt_client: Optional[mqtt.Client] = None
# Kezdeti állapot
last_state: dict = {"status": "unknown", "message": "Waiting for robot data..."}
mqtt_lock = threading.Lock()


class CommandIn(BaseModel):
    cmd: str


# ---- MQTT CALLBACK-EK ----

def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"[MQTT] Csatlakozva a brókerhez ({MQTT_HOST}:{MQTT_PORT}), kód: {reason_code}")
    # Feliratkozunk az állapotfrissítésekre
    client.subscribe(MQTT_STATE_TOPIC)
    print(f"[MQTT] Feliratkozva: {MQTT_STATE_TOPIC}")

def on_message(client, userdata, msg: mqtt.MQTTMessage):
    global last_state
    try:
        payload_str = msg.payload.decode("utf-8")
        
        # Ha a robot állapotot küld (JSON formátumban)
        if msg.topic == MQTT_STATE_TOPIC:
            try:
                # Megpróbáljuk JSON-ként értelmezni
                state_data = json.loads(payload_str)
                last_state = state_data
                # print(f"[TELEMETRY] Friss adat: {state_data}") # Debug-hoz bekapcsolható
            except json.JSONDecodeError:
                # Ha nem JSON jön (pl. sima szöveg vagy PONG válasz), azt is kezeljük
                last_state = {"raw_message": payload_str}
                print(f"[MQTT] Szöveges üzenet: {payload_str}")

    except Exception as e:
        print(f"[HIBA] Üzenet feldolgozása sikertelen: {e}")


# ---- FASTAPI ÉLETCIKLUS ----

@app.on_event("startup")
def startup_event():
    global mqtt_client
    print(f"[Startup] Csatlakozás MQTT-hez: {MQTT_HOST}:{MQTT_PORT} ...")
    
    # Kliens ID generálása
    mqtt_client = mqtt.Client(client_id="fastapi_backend_service")
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
        # Háttérszál indítása a hálózati forgalom kezelésére
        mqtt_client.loop_start()
        print("[Startup] MQTT kliens fut.")
    except Exception as e:
        print(f"[KRITIKUS HIBA] Nem sikerült csatlakozni az MQTT brokerhez: {e}")

@app.on_event("shutdown")
def shutdown_event():
    global mqtt_client
    print("[Shutdown] MQTT leállítása...")
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        mqtt_client = None
    print("[Shutdown] Kész.")


# ---- REST ENDPOINTOK ----

@app.post("/cmd")
async def send_command(cmd_in: CommandIn):
    """
    Parancs küldése a robotnak.
    Body: {"cmd": "sit"}
    """
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT nincs csatlakoztatva"}

    command = cmd_in.cmd.lower().strip()

    # Validáció
    if command not in VALID_COMMANDS:
        return {
            "status": "error",
            "message": f"Érvénytelen parancs! Támogatott: {', '.join(sorted(VALID_COMMANDS))}"
        }

    # Szálbiztos küldés
    with mqtt_lock:
        mqtt_client.publish(MQTT_CMD_TOPIC, command)

    print(f"[API] Parancs kiküldve: {command}")
    return {"status": "ok", "sent": command}


@app.get("/state")
async def get_state():
    """ 
    Visszaadja a robot legutolsó ismert telemetria adatait (JSON).
    """
    return {"status": "ok", "data": last_state}


@app.post("/ping")
async def send_ping():
    """
    Látencia méréshez: Kiküldi a PING_TEST jelet.
    A robot erre azonnal PONG-gal válaszol (de nem a ROS-on keresztül).
    """
    if mqtt_client is None:
        return {"status": "error", "message": "MQTT hiba"}

    with mqtt_lock:
        mqtt_client.publish(MQTT_PING_TOPIC, "PING_TEST")
    
    return {"status": "ping_sent"}
