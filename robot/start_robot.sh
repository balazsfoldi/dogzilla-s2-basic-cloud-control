#!/bin/bash

# --- KONFIGURÁCIÓ ---
# Melyik ROS verziót használod? (A logjaid alapján Foxy-nak tűnik)
ROS_DISTRO="foxy"
# ROS_DISTRO="humble" # Ha mégis Humble lenne, írd át erre

# Munkakönyvtár
WORK_DIR="/home/pi/Documents/balazs"

# Színek a loghoz
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}[INFO] Robot rendszer indítása...${NC}"

# 1. ROS Környezet betöltése
source /opt/ros/$ROS_DISTRO/setup.bash
# Ha van saját workspace-ed (pl. a Dogzilla gyári dolgaihoz), azt is be kell tölteni:
# source /home/pi/catkin_ws/install/setup.bash  <-- Ezt ellenőrizd, kell-e!

# 2. Hardver újraindítása (Biztos, ami biztos)
echo -e "${BLUE}[INFO] Hardver (YahboomStart) frissítése...${NC}"
sudo systemctl restart YahboomStart.service
sleep 5 # Várunk, hogy a hardver magához térjen

# 3. Node-ok indítása a háttérben (& jel a végén)
echo -e "${GREEN}[START] MQTT Híd indítása...${NC}"
cd $WORK_DIR
python3 mqtt_bridge_node.py &
PID_BRIDGE=$!

echo -e "${GREEN}[START] Dogzilla Agy indítása...${NC}"
python3 dogzilla_action_node.py &
PID_BRAIN=$!

echo -e "${GREEN}✅ Minden fut!${NC}"
echo -e "Híd PID: $PID_BRIDGE"
echo -e "Agy PID: $PID_BRAIN"
echo -e "A leállításhoz nyomj Ctrl+C-t, vagy futtasd a stop_robot.sh-t"

# Várakozás, hogy a script ne lépjen ki (így a Ctrl+C leállítja a folyamatokat)
wait $PID_BRIDGE $PID_BRAIN
