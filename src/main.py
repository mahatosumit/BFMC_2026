# ===================================== GENERAL IMPORTS ==================================

import sys
import time
import os
import psutil
import termios
import tty

# Pin to CPU cores
available_cores = list(range(psutil.cpu_count()))
psutil.Process(os.getpid()).cpu_affinity(available_cores)

sys.path.append(".")

from multiprocessing import Queue, Event
from src.utils.bigPrintMessages import BigPrint
from src.utils.messages.allMessages import (
    SpeedMotor,
    SteerMotor,
    StateChange
)
import logging

logging.basicConfig(level=logging.INFO)

# ===================================== PROCESS IMPORTS ==================================

from src.gateway.processGateway import processGateway
from src.dashboard.processDashboard import processDashboard
from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.data.Semaphores.processSemaphores import processSemaphores
from src.data.TrafficCommunication.processTrafficCommunication import processTrafficCommunication
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.statemachine.stateMachine import StateMachine
from src.statemachine.systemMode import SystemMode

# ===================================== KEYBOARD UTILS ==================================

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# ===================================== SHUTDOWN PROCESS ====================================

def shutdown_process(process, timeout=1):
    process.join(timeout)
    if process.is_alive():
        process.terminate()
        process.join(timeout)
        if process.is_alive():
            process.kill()
    print(f"Process {process} stopped")

# ===================================== PROCESS MANAGEMENT ==================================

def manage_process_life(process_class, process_instance, process_args, enabled, allProcesses):
    if enabled:
        if process_instance is None:
            process_instance = process_class(*process_args)
            allProcesses.append(process_instance)
            process_instance.start()
    else:
        if process_instance is not None and process_instance.is_alive():
            shutdown_process(process_instance)
            allProcesses.remove(process_instance)
            process_instance = None
    return process_instance

# ======================================== SETUP ====================================

print(BigPrint.PLEASE_WAIT.value)

allProcesses = []
allEvents = []

queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}

logging = logging.getLogger()

# ===================================== INITIALIZE ==================================

stateChangeSubscriber = messageHandlerSubscriber(queueList, StateChange, "lastOnly", True)
StateMachine.initialize_shared_state(queueList)

processGateway = processGateway(queueList, logging)
processGateway.start()

# ===================================== INITIALIZE PROCESSES ==================================

dashboard_ready = Event()
camera_ready = Event()
semaphore_ready = Event()
traffic_com_ready = Event()
serial_handler_ready = Event()

processDashboard = processDashboard(queueList, logging, dashboard_ready, debugging=False)
processCamera = processCamera(queueList, logging, camera_ready, debugging=False)
processSemaphore = processSemaphores(queueList, logging, semaphore_ready, debugging=False)
processTrafficCom = processTrafficCommunication(queueList, logging, 3, traffic_com_ready, debugging=False)
processSerialHandler = processSerialHandler(queueList, logging, serial_handler_ready, dashboard_ready, debugging=False)

allProcesses.extend([
    processCamera,
    processSemaphore,
    processTrafficCom,
    processSerialHandler,
    processDashboard
])

allEvents.extend([
    camera_ready,
    semaphore_ready,
    traffic_com_ready,
    serial_handler_ready,
    dashboard_ready
])

# ===================================== START PROCESSES ==================================

for proc in allProcesses:
    proc.daemon = True
    proc.start()

# ===================================== CONTROL STATE ==================================

MANUAL_MODE = True
current_speed = "0"
current_steer = "0"

print("[INFO] Keyboard control enabled")
print("[INFO] W/S = speed | A/D = steer | SPACE = stop | CTRL+C = exit")

# ===================================== MAIN LOOP ====================================

blocker = Event()

try:
    for event in allEvents:
        event.wait()

    StateMachine.initialize_starting_mode()

    print(BigPrint.C4_BOMB.value)
    print(BigPrint.PRESS_CTRL_C.value)

    while True:
        # ----------------------------------
        # BFMC state handling
        # ----------------------------------
        message = stateChangeSubscriber.receive()
        if message is not None:
            modeDictSemaphore = SystemMode[message].value["semaphore"]["process"]
            modeDictTrafficCom = SystemMode[message].value["traffic_com"]["process"]

            processSemaphore = manage_process_life(
                processSemaphores,
                processSemaphore,
                [queueList, logging, semaphore_ready, False],
                modeDictSemaphore["enabled"],
                allProcesses
            )

            processTrafficCom = manage_process_life(
                processTrafficCommunication,
                processTrafficCom,
                [queueList, logging, 3, traffic_com_ready, False],
                modeDictTrafficCom["enabled"],
                allProcesses
            )

        # ----------------------------------
        # MANUAL KEYBOARD CONTROL
        # ----------------------------------
        key = get_key()

        if key == 'w':
            current_speed = "130"
        elif key == 's':
            current_speed = "0"
        elif key == 'a':
            current_steer = "-20"
        elif key == 'd':
            current_steer = "20"
        elif key == 'x':
            current_steer = "0"
        elif key == ' ':
            current_speed = "0"
            current_steer = "0"

        queueList["General"].put((SpeedMotor, current_speed))
        queueList["General"].put((SteerMotor, current_steer))

        blocker.wait(0.05)

except KeyboardInterrupt:
    print("\n[SHUTDOWN] KeyboardInterrupt detected")

    for proc in reversed(allProcesses):
        proc.stop()

    processGateway.stop()

    for proc in reversed(allProcesses):
        shutdown_process(proc)

    shutdown_process(processGateway)
