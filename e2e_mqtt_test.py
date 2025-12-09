import paho.mqtt.client as mqtt
import time
import statistics
import csv
from datetime import datetime

# --- CONFIG: KÖZVETLEN MQTT (FASTAPI NÉLKÜL) ---

# A K8s Proxy IP-je és a NodePort (a rajzod alapján)
MQTT_BROKER = "10.19.8.25"  
MQTT_PORT = 31883           

# Témák
TOPIC_SEND = "robot/cmd"    # Ide küldjük a "ping"-et
TOPIC_RECV = "robot/state"  # Ide várjuk a "pong"-ot

MESSAGE_COUNT = 500  # Hány mérést végezzen
TIMEOUT = 5.0        # Max várakozás (mp)

# --- Változók ---
rtt_values = []
csv_data = [] 
lost_packets = 0
current_ping_start = 0
waiting_for_pong = False
last_latency = 0.0 

# --- MQTT CALLBACK-EK ---

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"✅ SIKERES CSATLAKOZÁS! ({MQTT_BROKER}:{MQTT_PORT})")
        # Feliratkozunk a válasz topikra
        client.subscribe(TOPIC_RECV)
        print(f"   Feliratkozva: {TOPIC_RECV}")
    else:
        print(f"❌ HIBA A CSATLAKOZÁSKOR! Kód: {rc}")

def on_message(client, userdata, msg):
    global waiting_for_pong, current_ping_start, rtt_values, last_latency
    
    try:
        payload = msg.payload.decode("utf-8")
        
        # Ha a válaszban benne van a "pong" és éppen várunk rá
        if "pong" in payload and waiting_for_pong:
            arrival_time = time.time()
            # RTT kiszámítása
            rtt_ms = (arrival_time - current_ping_start) * 1000
            
            last_latency = rtt_ms
            rtt_values.append(rtt_ms)
            print(f"   --> PONG ÉRKEZETT: {rtt_ms:.1f} ms")
            waiting_for_pong = False
            
    except Exception as e:
        print(f"Hiba az üzenet feldolgozásakor: {e}")

# --- FŐ PROGRAM ---

# Kliens létrehozása (paho-mqtt verziótól függően lehet, hogy kell a callback_api_version)
client = mqtt.Client("PC_Direct_Tester")
client.on_connect = on_connect
client.on_message = on_message

print("\n--- DIRECT MQTT E2E TESZT (FASTAPI BYPASS) ---")
print(f"Útvonal: PC -> K8s Proxy ({MQTT_BROKER}) -> Mosquitto -> ROBOT -> Mosquitto -> PC")

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    time.sleep(2) # Várjunk a kapcsolódásra
except Exception as e:
    print(f"KRITIKUS HIBA: Nem érhető el a bróker! {e}")
    exit(1)

try:
    for i in range(1, MESSAGE_COUNT + 1):
        timestamp_now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"[{i}/{MESSAGE_COUNT}] PING küldése...", end=" ", flush=True)
        
        current_ping_start = time.time()
        waiting_for_pong = True
        last_latency = 0.0
        status = "UNKNOWN"
        
        # 1. TRIGGER: Közvetlen MQTT publish
        try:
            # A bridge node-ban megírt logika szerint a 'ping' szóra 'pong'-gal válaszol
            msg_info = client.publish(TOPIC_SEND, "ping")
            
            if msg_info.rc != mqtt.MQTT_ERR_SUCCESS:
                print("MQTT SEND ERROR")
                lost_packets += 1
                status = "MQTT_PUB_ERR"
                waiting_for_pong = False
            
        except Exception as e:
            print(f"PUBLISH HIBA: {e}")
            lost_packets += 1
            status = "PUB_EXCEPTION"
            waiting_for_pong = False

        # 2. VÁRAKOZÁS a válaszra
        if waiting_for_pong:
            start_wait = time.time()
            while waiting_for_pong:
                # Timeout figyelése
                if time.time() - start_wait > TIMEOUT:
                    print(" -> IDŐTÚLLÉPÉS (Timeout)")
                    lost_packets += 1
                    waiting_for_pong = False
                    status = "TIMEOUT"
                    break
                time.sleep(0.01) # Kis pihenő a CPU kímélésére
        
        # Státusz frissítése a CSV-hez
        if status == "UNKNOWN":
            if last_latency > 0:
                status = "OK"
            else:
                status = "MISSING_PONG"

        # Adatsor mentése
        csv_data.append([i, timestamp_now, status, f"{last_latency:.2f}"])
        
        # Rövid szünet a mérések között (hogy ne terheljük túl a hálózatot)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nTeszt leállítva a felhasználó által.")

finally:
    client.loop_stop()
    client.disconnect()
    
    # --- STATISZTIKA ---
    if rtt_values:
        print("\n" + "="*45)
        print("        EREDMÉNYEK (DIRECT MQTT)")
        print("="*45)
        print(f"Mérések száma:       {len(rtt_values)} db")
        print(f"Átlagos RTT:         {statistics.mean(rtt_values):.1f} ms")
        print(f"Leggyorsabb:         {min(rtt_values):.1f} ms")
        print(f"Leglassabb:          {max(rtt_values):.1f} ms")
        print(f"Csomagvesztés:       {lost_packets} db")
        
        # Feltételezett infrastruktúra latency (ha van korábbi adatod)
        infra_latency = 314.0 
        try:
            overhead = statistics.mean(rtt_values) - infra_latency
            if overhead > 0:
                print("-" * 45)
                print(f"Robot + Wifi overhead: ~{overhead:.1f} ms (becsült)")
        except: pass
        print("="*45)

    # --- CSV MENTÉS ---
    filename = f"direct_mqtt_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(["Sorszam", "Idobelyeg", "Statusz", "Latency_ms"])
            writer.writerows(csv_data)
        print(f"\nAdatok elmentve: {filename}")
    except Exception as e:
        print(f"Hiba a mentésnél: {e}")
