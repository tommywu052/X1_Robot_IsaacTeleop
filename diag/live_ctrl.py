#!/usr/bin/env python3
"""Live change-logger for controller_data. Prints a line whenever a stick or
button changes, so we can see exactly what the emulator injects in real time.
Run ~20s: push RIGHT stick a few times, then press left Y once."""
import time
import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import ByteMultiArray

WATCH = [
    "right_thumbstick", "left_thumbstick",
    "left_secondary_click", "left_primary_click",
    "right_secondary_click", "right_primary_click",
    "left_trigger_value", "right_trigger_value",
    "left_squeeze_value", "right_squeeze_value",
]

def norm(v):
    if isinstance(v, (list, tuple)):
        return tuple(round(float(x), 3) for x in v)
    try:
        return round(float(v), 3)
    except Exception:
        return v

class L(Node):
    def __init__(self):
        super().__init__("live_ctrl")
        self.prev = {}
        self.n = 0
        self.t0 = time.time()
        best = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(ByteMultiArray, "/xr_teleop/controller_data", self._c, best)

    def _c(self, m):
        raw = b"".join(x if isinstance(x, (bytes, bytearray)) else bytes([x & 0xFF]) for x in m.data)
        try:
            d = msgpack.unpackb(raw, raw=False)
        except Exception:
            return
        self.n += 1
        t = time.time() - self.t0
        for k in WATCH:
            if k not in d:
                continue
            cur = norm(d[k])
            if k not in self.prev:
                self.prev[k] = cur
                continue
            # threshold for sticks
            changed = False
            if isinstance(cur, tuple):
                changed = any(abs(a - b) > 0.05 for a, b in zip(cur, self.prev[k]))
            else:
                changed = abs((cur or 0) - (self.prev[k] or 0)) > 0.05
            if changed:
                print("[t=%5.1fs] %-22s %s -> %s" % (t, k, self.prev[k], cur))
                self.prev[k] = cur

def main():
    rclpy.init(); n = L()
    end = time.time() + 20
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    print("--- done. controller_data msgs=%d in 20s (%.1f Hz) ---" % (n.n, n.n / 20.0))
    if n.n and not any(True for _ in []):
        pass
    print("watched-field first-seen values:")
    for k in WATCH:
        if k in n.prev:
            print("   %-22s %s" % (k, n.prev[k]))
    n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
