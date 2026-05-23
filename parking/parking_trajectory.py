import csv
from .parking_config import (
    MANEUVER_SPEED_PWM,
    MANEUVER_STEER_RIGHT_DEG, MANEUVER_STEER_LEFT_DEG,
    MANEUVER_T1_S, MANEUVER_T2_S, MANEUVER_T3_S,
)

_LOOP_HZ = 20   # must match main.py LOOP_HZ


def _secs_to_frames(seconds):
    return max(1, round(seconds * _LOOP_HZ))


class ParkingTrajectory:
    def __init__(self,
                 left_csv="parking/left_parallel_parking.csv",
                 right_csv="parking/right_parallel_parking.csv"):
        self.left_csv_path  = left_csv
        self.right_csv_path = right_csv
        self.trajectory_points = None

    # ── CSV loader ────────────────────────────────────────────
    def _load_csv(self, file_path):
        """
        CSV format expected:  time(s), steering(deg), speed(fraction ±1)
          time   = step duration in seconds (if first row is 0.0, treated as delta)
          speed  > 0 → forward,  speed < 0 → reverse
        Output keys match main.py playback: speed, steer, pwm, direction, duration_fr
        """
        trajectory = []
        try:
            with open(file_path, mode='r') as f:
                reader = csv.reader(f)
                next(reader, None)   # skip header

                rows = []
                for row in reader:
                    if len(row) >= 3:
                        rows.append((float(row[0]), float(row[1]), float(row[2])))

            prev_t = None
            for t_raw, steer, spd_frac in rows:
                if prev_t is None:
                    dt_s = max(0.05, t_raw) if t_raw > 0.0 else 0.05
                else:
                    delta = t_raw - prev_t
                    dt_s  = max(0.05, delta) if delta > 0.001 else max(0.05, t_raw)
                prev_t = t_raw

                direction = -1 if spd_frac < 0 else 1
                speed_pwm = abs(spd_frac) * 150.0   # fraction → PWM (base=150)

                trajectory.append({
                    "speed":       speed_pwm,
                    "steer":       steer,
                    "pwm":         0.0,
                    "direction":   direction,
                    "duration_fr": _secs_to_frames(dt_s),
                })

        except FileNotFoundError:
            print(f"[Parking] CSV not found: {file_path}")
        except Exception as e:
            print(f"[Parking] CSV load error ({file_path}): {e}")

        return trajectory

    # ── Open-loop fallback ────────────────────────────────────
    def _openloop_trajectory(self, side="right"):
        """
        Programmatic parallel-parking maneuver when no CSV is present.
        Tune MANEUVER_T1_S / T2_S / T3_S in parking_config.py on the real track.

        RIGHT slot:
          1. Reverse + full-right lock  (steer into slot)
          2. Reverse + full-left  lock  (straighten)
          3. Creep forward + straight   (centre in slot)

        LEFT slot: mirrors the steering.
        """
        if side == "right":
            s1, s2 = MANEUVER_STEER_RIGHT_DEG, MANEUVER_STEER_LEFT_DEG
        else:
            s1, s2 = MANEUVER_STEER_LEFT_DEG, MANEUVER_STEER_RIGHT_DEG

        spd = MANEUVER_SPEED_PWM
        return [
            {"speed": spd, "steer": s1,  "pwm": 0.0, "direction": -1,
             "duration_fr": _secs_to_frames(MANEUVER_T1_S)},
            {"speed": spd, "steer": s2,  "pwm": 0.0, "direction": -1,
             "duration_fr": _secs_to_frames(MANEUVER_T2_S)},
            {"speed": spd, "steer": 0.0, "pwm": 0.0, "direction":  1,
             "duration_fr": _secs_to_frames(MANEUVER_T3_S)},
        ]

    # ── Public loaders ────────────────────────────────────────
    def load_left_trajectory(self):
        traj = self._load_csv(self.left_csv_path)
        if not traj:
            print("[Parking] LEFT CSV empty/missing — using open-loop fallback")
            traj = self._openloop_trajectory("left")
        self.trajectory_points = traj
        print(f"[Parking] Left trajectory ready: {len(traj)} steps")
        return traj

    def load_right_trajectory(self):
        traj = self._load_csv(self.right_csv_path)
        if not traj:
            print("[Parking] RIGHT CSV empty/missing — using open-loop fallback")
            traj = self._openloop_trajectory("right")
        self.trajectory_points = traj
        print(f"[Parking] Right trajectory ready: {len(traj)} steps")
        return traj
