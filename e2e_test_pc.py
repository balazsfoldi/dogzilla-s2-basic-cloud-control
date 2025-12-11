import paho.mqtt.client as mqtt
import requests
import time
import statistics
import csv
from datetime import datetime

# --- CONFIG: ÉLES RENDSZER ---
# 1. Trigger: HTTP kérés a FastAPI-nak
API_URL = "http://10.19.8.25:30081/ping"

# 2. Visszajelzés: MQTT a Robot felől
MQTT_BROKER = "10.19.8.25"
MQTT_PORT = 31883
TOPIC_RECV = "dogzilla/control/pong"

MESSAGE_COUNT = 200
TIMEOUT = 5.0

# Változók
rtt_values = []
csv_data = [] # Adatok tárolása a mentéshez
lost_packets = 0
current_ping_start = 0
last_latency = 0.0 # Segédváltozó az aktuális méréshez

def on_connect(client, userdata, flags, rc):
    print("PC: MQTT Csatlakozva. Várakozás a robot válaszára...")
    client.subscribe(TOPIC_RECV)

def on_message(client, userdata, msg):
    global waiting_for_pong, current_ping_start, rtt_values, last_latency
    
    payload = msg.payload.decode()
    
    if payload == "PONG_RESPONSE" and waiting_for_pong:
        arrival_time = time.time()
        # Teljes köridő kiszámítása
        rtt_ms = (arrival_time - current_ping_start) * 1000
        
        last_latency = rtt_ms # Elmentjük a CSV íráshoz
        rtt_values.append(rtt_ms)
        print(f"   --> ROBOT VÁLASZOLT: {rtt_ms:.1f} ms")
        waiting_for_pong = False

# MQTT Indítása
client = mqtt.Client(client_id="PC_Master_Controller")
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

print("\n--- TELJES KÖR (END-TO-END) TESZT + CSV MENTÉS ---")
print("Útvonal: PC -> FastAPI -> Cloud MQTT -> FIZIKAI ROBOT -> Cloud MQTT -> PC")
time.sleep(2)

try:
    for i in range(1, MESSAGE_COUNT + 1):
        timestamp_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{i}/{MESSAGE_COUNT}] Parancs küldése...", end=" ", flush=True)
        
        current_ping_start = time.time()
        waiting_for_pong = True
        last_latency = 0.0
        status = "UNKNOWN"
        
        # 1. Kérés küldése a FastAPI-nak
        try:
            resp = requests.post(API_URL, timeout=3)
            if resp.status_code == 200:
                print("API OK", end="")
            else:
                print(f"API HIBA ({resp.status_code})")
                lost_packets += 1
                waiting_for_pong = False
                status = f"API_ERR_{resp.status_code}"
                csv_data.append([i, timestamp_now, status, 0])
                continue
        except Exception as e:
            print(f"HÁLÓZATI HIBA: {e}")
            lost_packets += 1
            waiting_for_pong = False
            status = "NET_EXCEPTION"
            csv_data.append([i, timestamp_now, status, 0])
            continue

        # 2. Várakozás a fizikai robot válaszára
        start_wait = time.time()
        while waiting_for_pong:
            if time.time() - start_wait > TIMEOUT:
                print(" -> IDŐTÚLLÉPÉS (A robot nem válaszolt időben)")
                lost_packets += 1
                waiting_for_pong = False
                status = "ROBOT_TIMEOUT"
                break
            time.sleep(0.01)
        
        # Státusz véglegesítése a CSV-hez
        if status == "UNKNOWN":
            if not waiting_for_pong and last_latency > 0:
                status = "OK"
            else:
                status = "TIMEOUT_UNKNOWN"
        
        # Adatsor mentése
        csv_data.append([i, timestamp_now, status, f"{last_latency:.2f}"])
        
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTeszt leállítva.")

finally:
    client.loop_stop()
    client.disconnect()
    
    # --- EREDMÉNYEK KONZOLON ---
    if rtt_values:
        print("\n" + "="*45)
        print("       FIZIKAI ROBOT TELJES KÉSLELTETÉS")
        print("="*45)
        print(f"Mérések száma:       {len(rtt_values)} db")
        print(f"Átlagos reakcióidő:  {statistics.mean(rtt_values):.1f} ms")
        print(f"Leggyorsabb:         {min(rtt_values):.1f} ms")
        print(f"Leglassabb:          {max(rtt_values):.1f} ms")
        print(f"Csomagvesztés:       {lost_packets} db")
        print("-" * 45)
        print("Infrastruktúra (előző mérés): ~314 ms")
        try:
            robot_overhead = statistics.mean(rtt_values) - 314
            print(f"Robot hozzáadott ideje:       ~{robot_overhead:.1f} ms")
        except:
            pass
        print("="*45)

    # --- CSV MENTÉS ---
    filename = f"robot_full_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(["Sorszam", "Idobelyeg", "Statusz", "Latency_ms"])
            writer.writerows(csv_data)
        print(f"\nRészletes adatok elmentve: {filename}")
    except Exception as e:
        print(f"Hiba a fájlmentésnél: {e}")