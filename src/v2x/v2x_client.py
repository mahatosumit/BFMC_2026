import socket
import json
import threading
import time
import logging

log = logging.getLogger(__name__)


class V2XClient(threading.Thread):
    """Lightweight TCP client that speaks the BFMC V2X protocol."""

    def __init__(self, host="127.0.0.1", port=5000, frequency=0.5):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.frequency = frequency
        self.running = False
        self._lock = threading.Lock()

        # Car state (updated externally by main loop)
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_speed = 0.0
        self.sock = None

    def update_state(self, x, y, yaw, speed):
        """Called by main loop to update telemetry values."""
        with self._lock:
            self.car_x = float(x)
            self.car_y = float(y)
            self.car_yaw = float(yaw)
            self.car_speed = float(speed)

    def _connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            print(f"[V2X] Connected to server {self.host}:{self.port}")
            self._last_conn_error = False
            return True
        except Exception as e:
            if not getattr(self, '_last_conn_error', False):
                print(f"[V2X] Server offline at {self.host}:{self.port}. Waiting for it to start...")
                self._last_conn_error = True
            self.sock = None
            return False

    def _send(self, msg_dict):
        if not self.sock:
            return
        try:
            data = json.dumps(msg_dict).encode()
            self.sock.sendall(data)
        except Exception as e:
            print(f"[V2X] Send failed: {e}, will reconnect")
            try:
                self.sock.close()
            except:
                pass
            self.sock = None  # will reconnect next cycle

    def run(self):
        self.running = True
        log.info("[V2X] Client thread started")
        while self.running:
            # Reconnect if needed
            if not self.sock:
                if not self._connect():
                    time.sleep(2.0)
                    continue

            # Snapshot current state
            with self._lock:
                x, y = self.car_x, self.car_y
                yaw = self.car_yaw
                speed = self.car_speed

            # Push position
            self._send({
                "reqORinfo": "info",
                "type": "devicePos",
                "value1": round(x, 4),
                "value2": round(y, 4)
            })
            # Push rotation (yaw)
            self._send({
                "reqORinfo": "info",
                "type": "deviceRot",
                "value1": round(yaw, 2)
            })
            # Push speed
            self._send({
                "reqORinfo": "info",
                "type": "deviceSpeed",
                "value1": round(speed, 1)
            })

            time.sleep(self.frequency)

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        log.info("[V2X] Client stopped")
