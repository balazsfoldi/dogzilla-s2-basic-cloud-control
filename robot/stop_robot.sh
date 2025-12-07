#!/bin/bash
echo "🛑 Robot folyamatok leállítása..."

# Megkeressük és leállítjuk a python folyamatokat név alapján
pkill -f "python3 mqtt_bridge_node.py"
pkill -f "python3 dogzilla_action_node.py"

echo "✅ Leállítva."
