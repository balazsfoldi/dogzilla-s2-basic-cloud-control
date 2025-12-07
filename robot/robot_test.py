import paho.mqtt.client as mqtt
import time

# --- CONFIG: ROBOT BEÁLLÍTÁSA ---
# A robotnak a felhő külső IP-jét kell elérnie
BROKER = "155.98.37.82"
PORT = 31883   # A NodePort, amin az MQTT kint van
TOPIC_IN = "dogzilla/control/ping"
TOPIC_OUT = "dogzilla/control/pong"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ ROBOT: Csatlakozva a Felhőhöz! ({BROKER}:{PORT})")
        client.subscribe(TOPIC_IN)
    else:
        print(f"❌ ROBOT: Hiba a csatlakozáskor (Kód: {rc})")

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    
    if payload == "PING_TEST":
        # Amint megjön a parancs a FastAPI-tól, azonnal visszalőjük
        # Itt történne a valódi mozgás (pl. séta) indítása is
        client.publish(TOPIC_OUT, "PONG_RESPONSE")
        print(f"[PING] Parancs fogadva -> [PONG] Válasz küldve")

client = mqtt.Client(client_id="Dogzilla_Physical_Robot")
client.on_connect = on_connect
client.on_message = on_message

print("--- FIZIKAI ROBOT RENDSZER INDÍTÁSA ---")
print("Várakozás a parancsokra...")

try:
    client.connect(BROKER, PORT, 60)
    client.loop_forever()
except Exception as e:
    print(f"KRITIKUS HIBA: Nem érem el a szervert! {e}")