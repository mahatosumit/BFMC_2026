import threading
import time
import math

try:
    import smbus
    _SMBUS_AVAILABLE = True
except ImportError:
    _SMBUS_AVAILABLE = False
    print("[IMU] smbus module not found. IMU telemetry will mock zeros.")

class IMUSensor(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.yaw_deg = 0.0
        self.running = False
        self.bus = None
        self.is_calibrated = False
        
        self.BNO_ADDR = 0x28
        self.BNO_ADDR_ALT = 0x29
        self.OPR_MODE = 0x3D
        self.CHIP_ID = 0x00
        self.QUAT_W_LSB = 0x20
        self.CALIB_STAT = 0x35
        
        if _SMBUS_AVAILABLE:
            try:
                self.bus = smbus.SMBus(1)
            except Exception as e:
                print(f"[IMU] Failed to open smbus: {e}")

    def safe_read8(self, reg, retries=5):
        if not self.bus: return 0
        for _ in range(retries):
            try:
                return self.bus.read_byte_data(self.BNO_ADDR, reg)
            except OSError as e:
                if e.errno in (121, 110):  # Remote I/O error or Connection timed out
                    time.sleep(0.01)
                    continue
                else:
                    time.sleep(0.01)
                    continue
        print(f"[IMU WARNING] Could not read register {hex(reg)} after {retries} retries.")
        return 0

    def safe_write8(self, reg, value, retries=5):
        if not self.bus: return
        for _ in range(retries):
            try:
                self.bus.write_byte_data(self.BNO_ADDR, reg, value)
                return
            except OSError as e:
                if e.errno in (121, 110):
                    time.sleep(0.01)
                    continue
                else:
                    time.sleep(0.01)
                    continue
        print(f"[IMU WARNING] Could not write {hex(value)} to {hex(reg)}")

    def safe_read16(self, reg):
        if not self.bus: return 0
        lsb = self.safe_read8(reg)
        msb = self.safe_read8(reg + 1)
        value = (msb << 8) | lsb
        if value > 32767:
            value -= 65536
        return value

    def run(self):
        if not self.bus:
            print("[IMU] Hardware inactive, thread stopping.")
            return

        print("[IMU] Hardware initialized. Searching for BNO055...")
        time.sleep(1)

        # Detect BNO055 on Primary or Alt address
        try:
            chip = self.safe_read8(self.CHIP_ID)
            if chip != 0xA0:
                print(f"[IMU] BNO055 not at 0x28. Checking alternative address 0x29...")
                self.BNO_ADDR = self.BNO_ADDR_ALT
                chip = self.safe_read8(self.CHIP_ID)
                if chip != 0xA0:
                    print(f"[IMU] BNO055 not detected on either address. Disabling.")
                    return
            print(f"[IMU] BNO055 found at {hex(self.BNO_ADDR)}! Chip ID: {hex(chip)}")

            # CONFIG Mode
            self.safe_write8(self.OPR_MODE, 0x00)
            time.sleep(0.05)
            # NDOF Mode (Absolute Orientation Fusion)
            self.safe_write8(self.OPR_MODE, 0x0C)
            time.sleep(0.1)
            print("[IMU] NDOF mode activated.")

            self.running = True
            self.start_time = time.time()
            
            calib_delay = 0
            
            while self.running:
                try:
                    # 1. Read Calibration Status every ~1 second (every 20th loop)
                    calib_delay += 1
                    if calib_delay >= 20:
                        calib_delay = 0
                        calib = self.safe_read8(self.CALIB_STAT)
                        sys = (calib >> 6) & 0x03
                        gyr = (calib >> 4) & 0x03
                        acc = (calib >> 2) & 0x03
                        mag = calib & 0x03
                        
                        if sys >= 2 and gyr >= 2 and mag >= 2:
                            self.is_calibrated = True
                        elif time.time() - self.start_time > 5.0:
                            if not getattr(self, "_forced_calib_msg", False):
                                print("[IMU CALIB] 5-second timeout reached. Forcing calibration PASS.")
                                self._forced_calib_msg = True
                            self.is_calibrated = True
                        else:
                            self.is_calibrated = False
                            print(f"[IMU CALIB] Status - SYS:{sys} (Requires >0) | GYR:{gyr} | ACC:{acc} | MAG:{mag} (3 is fully calibrated)")

                    # 2. Read Quaternions
                    qw = self.safe_read16(self.QUAT_W_LSB) / 16384.0
                    qx = self.safe_read16(self.QUAT_W_LSB + 2) / 16384.0
                    qy = self.safe_read16(self.QUAT_W_LSB + 4) / 16384.0
                    qz = self.safe_read16(self.QUAT_W_LSB + 6) / 16384.0

                    # Convert to Euler Yaw
                    yaw = math.atan2(
                        2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz)
                    )

                    self.yaw_deg = math.degrees(yaw)
                    time.sleep(0.05)   # 20 Hz loop
                except Exception as loop_e:
                    print(f"[IMU] Non-fatal loop exception: {loop_e}. Recovering...")
                    time.sleep(0.1)
                
        except Exception as e:
            print(f"[IMU] Fatal exception in loop: {e}")
            self.running = False

    def stop(self):
        self.running = False

    def get_yaw(self):
        return self.yaw_deg
