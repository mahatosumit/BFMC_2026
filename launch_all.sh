#!/bin/bash

echo "========================================="
echo "   BFMC 2026 Autonomous Stack Launcher   "
echo "========================================="

# Get the absolute path to the directory where this script is located
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

echo "[1/3] Launching TrafficCommunication Server..."
# Launch V2X core TCP/UDP network server
lxterminal --title="V2X Core Server" -e "bash -c 'cd $BASE_DIR/servers/trafficCommunicationServer && python TrafficCommunication.py; exec bash'" &

# Give it a second to bind to port 5000
sleep 1

echo "[2/3] Launching Semaphore and Car Simulator..."
# Launch UDP broadcast simulators
lxterminal --title="Semaphore Simulator" -e "bash -c 'cd $BASE_DIR/servers/carsAndSemaphoreStreamSIM && python udpStreamSIM.py; exec bash'" &

sleep 1

echo "[3/3] Launching Main Car Application..."
# Launch the Tkinter dashboard, camera, and autonomous pipeline
lxterminal --title="BFMC 2026 Dashboard" -e "bash -c 'cd $BASE_DIR && python main.py; exec bash'" &

echo "All services launched in separate windows!"
