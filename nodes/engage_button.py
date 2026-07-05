#!/usr/bin/env python3
"""Toggle pink_arm_ik engage from an XR controller button.

The frontend publishes /xr_teleop/controller_data as a MessagePack-encoded map
(std_msgs/ByteMultiArray). Fields include per-hand button clicks:
  left/right_primary_click   (A / X)
  left/right_secondary_click (B / Y)
  left/right_menu_click      (menu / start button)
  left/right_thumbstick_click
Values are float64 (0.0/1.0) or bool. On a rising edge of the selected button
we flip engaged and call the pink_arm_ik ~/engage SetBool service.

Default button = left_menu_click (the "start"/menu button). Note the IWER
browser emulator can pin right_primary_click at 1.0, so avoid that field there.
"""
import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import ByteMultiArray
from std_srvs.srv import SetBool


def _truthy(v):
    if isinstance(v, bool):
        return v
    try:
        return float(v) > 0.5
    except (TypeError, ValueError):
        return False


class EngageButton(Node):
    def __init__(self):
        super().__init__("engage_button")
        self.declare_parameter("button_field", "left_menu_click")
        self.declare_parameter("engage_service", "/pink_arm_ik/engage")
        self.declare_parameter("start_engaged", True)  # assumed current pink state
        self.declare_parameter("debounce_s", 0.4)

        self.field = self.get_parameter("button_field").value
        self.state = bool(self.get_parameter("start_engaged").value)
        self.debounce = float(self.get_parameter("debounce_s").value)
        self.prev = False
        self.last_toggle = 0.0
        self.warned = False

        best = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(ByteMultiArray, "/xr_teleop/controller_data", self._cb, best)
        self.cli = self.create_client(SetBool, self.get_parameter("engage_service").value)
        self.get_logger().info("engage_button up: watching '%s' (assumed engaged=%s)"
                               % (self.field, self.state))

    def _cb(self, msg):
        raw = b"".join(x if isinstance(x, (bytes, bytearray)) else bytes([x & 0xFF])
                       for x in msg.data)
        try:
            d = msgpack.unpackb(raw, raw=False)
        except Exception:  # noqa: BLE001
            return
        if self.field not in d:
            if not self.warned:
                self.get_logger().warn("field '%s' not in controller_data; keys=%s"
                                       % (self.field, list(d.keys())))
                self.warned = True
            return
        pressed = _truthy(d[self.field])
        now = self.get_clock().now().nanoseconds * 1e-9
        if pressed and not self.prev and (now - self.last_toggle) > self.debounce:
            self.last_toggle = now
            self.state = not self.state
            self._send(self.state)
        self.prev = pressed

    def _send(self, engage):
        if not self.cli.service_is_ready():
            self.cli.wait_for_service(timeout_sec=1.0)
        req = SetBool.Request()
        req.data = bool(engage)
        self.cli.call_async(req)
        self.get_logger().info("button -> engage=%s" % engage)


def main():
    rclpy.init()
    node = EngageButton()
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
