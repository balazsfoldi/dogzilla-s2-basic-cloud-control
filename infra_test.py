import paho.mqtt.client as mqtt
import requests
import time
import statistics
import csv
from datetime import datetime

# --- KONFIGURÁCIÓ (A Te Cloud Címeiddel) ---

# 1. API: Ide küldjük a HTTP kérést (Trigger)
API_URL = "http://155.98.37.82:30081/ping"

# 2. MQTT: Innen hallgatjuk vissza (ugyanaz a topic, ahova a FastAPI ír!)
MQTT_BROKER = "155.98.37.82"
MQTT_PORT = 31883
TOPIC_LISTEN = "dogzilla/control/ping"  # Figyeljük, mit küld a FastAPI

MESSAGE_COUNT = 500
TIMEOUT = 3.0

# Változók
rtt_values = []
csv_data = []  # Ebben tároljuk a részletes adatokat a CSV-hez
lost_packets = 0
current_ping_start = 0
waiting_for_message = False
last_latency = 0.0 # Segédváltozó az aktuális mérés átadásához

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"MQTT Csatlakozva ({MQTT_BROKER}:{MQTT_PORT})")
        # Feliratkozunk arra, ahova a FastAPI üzen
        client.subscribe(TOPIC_LISTEN)
    else:
        print(f"MQTT Hiba kód: {rc}")

def on_message(client, userdata, msg):
    global waiting_for_message, current_ping_start, rtt_values, last_latency
    
    payload = msg.payload.decode()
    
    # Ha mi indítottuk a mérést, és megjött a PING_TEST üzenet
    if payload == "PING_TEST" and waiting_for_message:
        arrival_time = time.time()
        # Kiszámoljuk az eltelt időt
        latency_ms = (arrival_time - current_ping_start) * 1000
        
        last_latency = latency_ms # Elmentjük az aktuális értéket a CSV-hez
        rtt_values.append(latency_ms)
        print(f"   --> Visszaért: {latency_ms:.1f} ms")
        waiting_for_message = False

# MQTT Indítása
client = mqtt.Client(client_id="Infra_Tester_PC")
client.on_connect = on_connect
client.on_message = on_message

print("\n--- INFRASTRUKTÚRA TESZT (Robot nélkül) + CSV Mentés ---")
print("Útvonal: PC -> HTTP -> FastAPI -> MQTT -> PC")
print(f"Cél API: {API_URL}")
print(f"Figyelt Topic: {TOPIC_LISTEN}")

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    time.sleep(1) # Várakozás a kapcsolatra

    for i in range(1, MESSAGE_COUNT + 1):
        timestamp_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{i}/{MESSAGE_COUNT}] HTTP Start...", end=" ", flush=True)
        
        current_ping_start = time.time()
        waiting_for_message = True
        last_latency = 0.0 # Nullázzuk a mérés előtt
        status = "UNKNOWN"
        
        # 1. HTTP Kérés küldése (ez indítja a lavinát)
        try:
            resp = requests.post(API_URL, timeout=2)
            if resp.status_code == 200:
                print("OK (API)", end="")
            else:
                print(f"HIBA (HTTP {resp.status_code})")
                waiting_for_message = False
                lost_packets += 1
                status = f"HTTP_ERR_{resp.status_code}"
                csv_data.append([i, timestamp_now, status, 0])
                continue
        except Exception as e:
            print(f"API Hiba: {e}")
            waiting_for_message = False
            lost_packets += 1
            status = "API_EXCEPTION"
            csv_data.append([i, timestamp_now, status, 0])
            time.sleep(1)
            continue

        # 2. Várakozás, hogy az MQTT-n megjelenjen az üzenet
        start_wait = time.time()
        while waiting_for_message:
            if time.time() - start_wait > TIMEOUT:
                print(" -> IDŐTÚLLÉPÉS (Nem ért vissza MQTT-n)")
                lost_packets += 1
                waiting_for_message = False
                status = "MQTT_TIMEOUT"
                break
            time.sleep(0.005) # Nagyon pici szünet a pontosságért
        
        # Ha nem volt timeout és a loop kilépett, akkor sikerült (vagy timeoutolt)
        if status == "UNKNOWN": # Ha még nem állítottuk be hibára
            if not waiting_for_message and last_latency > 0:
                status = "OK"
            else:
                status = "MQTT_TIMEOUT" # Biztonsági ellenőrzés

        # Adatok mentése a listába
        csv_data.append([i, timestamp_now, status, f"{last_latency:.2f}"])
        
        time.sleep(0.2) # Rövid pihenő a köv. mérés előtt

except KeyboardInterrupt:
    print("\nTeszt leállítva.")

finally:
    client.loop_stop()
    client.disconnect()
    
    # --- EREDMÉNYEK STATISZTIKA ---
    if rtt_values:
        print("\n" + "="*40)
        print("       FELHŐ INFRASTRUKTÚRA KÉSLELTETÉS")
        print("="*40)
        print(f"Mérések száma:       {len(rtt_values)} db")
        print(f"Átlagos idő:         {statistics.mean(rtt_values):.1f} ms")
        print(f"Minimum (Legjobb):   {min(rtt_values):.1f} ms")
        print(f"Maximum (Legrosszabb): {max(rtt_values):.1f} ms")
        print(f"Jitter (Ingadozás):  {statistics.stdev(rtt_values):.1f} ms")
        print(f"Csomagvesztés:       {lost_packets} db")
        print("-" * 40)
    
    # --- CSV MENTÉS ---
    filename = f"network_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file, delimiter=';')
            # Fejléc
            writer.writerow(["Sorszam", "Idobelyeg", "Statusz", "Késleltetés_ms"])
            # Adatok
            writer.writerows(csv_data)
        
        print(f"Részletes adatok elmentve ide: {filename}")
    except Exception as e:
        print(f"Hiba a CSV mentésekor: {e}")
        
    print("="*40)