#!/usr/bin/env python3
"""Print which controller_data button/analog fields are active, so we can
identify the emulator's 'menu' button. Logs a line whenever any *_click goes
high or a trigger/squeeze crosses 0.5."""
import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import ByteMultiArray

CLICKS = [
    "left_primary_click", "right_primary_click",
    "left_secondary_click", "right_secondary_click",
    "left_menu_click", "right_menu_click",
    "left_thumbstick_click", "right_thumbstick_click",
]


def _f(v):
    if isinstance(v, bool):
        return 1.0 if v else 0.0
    try:
        return float(v)
    except (TypeError, ValueError):
        return 0.0


class Detect(Node):
    def __init__(self):
        super().__init__("btn_detect")
        best = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(ByteMultiArray, "/xr_teleop/controller_data", self._cb, best)
        self.prev = {}
        self.get_logger().info("press a button (esp. the menu/start button) ...")

    def _cb(self, msg):
        raw = b"".join(x if isinstance(x, (bytes, bytearray)) else bytes([x & 0xFF])
                       for x in msg.data)
        try:
            d = msgpack.unpackb(raw, raw=False)
        except Exception:  # noqa: BLE001
            return
        for k in CLICKS:
            cur = _f(d.get(k, 0.0)) > 0.5
            if cur and not self.prev.get(k, False):
                self.get_logger().info("DOWN  %s" % k)
            elif not cur and self.prev.get(k, False):
                self.get_logger().info("up    %s" % k)
            self.prev[k] = cur


def main():
    rclpy.init()
    node = Detect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
