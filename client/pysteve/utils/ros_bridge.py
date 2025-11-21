"""
ROS2 bridge for STEVE valve integration.
"""

import time
import threading
from typing import Optional, Dict, Any

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, Float64MultiArray, String
    from sensor_msgs.msg import JointState
except ImportError:
    raise ImportError(
        "ROS2 required. Install with: pip install pysteve[ros] "
        "or ensure ROS2 environment is sourced."
    )

from pysteve.core.client import SteveClient
from pysteve.core.streaming import SteveStreamer


class SteveRosBridge(Node):
    """
    ROS2 bridge for STEVE valve device.
    
    Publishes valve state to ROS topics and accepts commands.
    
    Topics Published:
        ~/state (sensor_msgs/JointState): Joint state with position, velocity, effort
        ~/position (std_msgs/Float64): Position in radians
        ~/velocity (std_msgs/Float64): Velocity in rad/s
        ~/torque (std_msgs/Float64): Torque in N·m
        ~/status (std_msgs/String): JSON status string
    
    Topics Subscribed:
        ~/cmd_torque (std_msgs/Float64): Torque command
        ~/cmd_position (std_msgs/Float64): Position command (not supported)
    
    Args:
        device_ip: STEVE device IP address
        node_name: ROS node name (default: "steve_valve")
        publish_rate_hz: Publishing rate (default: 50)
        api_key: STEVE API key
    
    Example:
        >>> import rclpy
        >>> from pysteve.utils.ros_bridge import SteveRosBridge
        >>> 
        >>> rclpy.init()
        >>> bridge = SteveRosBridge(device_ip="192.168.1.100")
        >>> bridge.connect()
        >>> bridge.start()
        >>> 
        >>> try:
        ...     rclpy.spin(bridge)
        ... except KeyboardInterrupt:
        ...     pass
        >>> 
        >>> bridge.disconnect()
        >>> rclpy.shutdown()
    """

    def __init__(
        self,
        device_ip: str,
        node_name: str = "steve_valve",
        publish_rate_hz: int = 50,
        api_key: str = "steve-valve-2025",
    ):
        super().__init__(node_name)

        self.device_ip = device_ip
        self.publish_rate_hz = publish_rate_hz
        self.api_key = api_key

        # STEVE clients
        self.client: Optional[SteveClient] = None
        self.streamer: Optional[SteveStreamer] = None

        # Publishers
        self.state_pub = self.create_publisher(JointState, "~/state", 10)
        self.position_pub = self.create_publisher(Float64, "~/position", 10)
        self.velocity_pub = self.create_publisher(Float64, "~/velocity", 10)
        self.torque_pub = self.create_publisher(Float64, "~/torque", 10)
        self.status_pub = self.create_publisher(String, "~/status", 10)

        # Subscribers
        self.torque_cmd_sub = self.create_subscription(
            Float64, "~/cmd_torque", self._torque_cmd_callback, 10
        )

        # Publishing timer
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish_callback)

        # State
        self._last_sample: Optional[Dict[str, Any]] = None

        self.get_logger().info(f"Initialized STEVE ROS bridge for {device_ip}")

    def connect(self) -> None:
        """Connect to STEVE device."""
        self.get_logger().info("Connecting to STEVE device...")

        self.client = SteveClient(self.device_ip, api_key=self.api_key, auto_reconnect=True)
        self.client.connect()

        # Start streaming
        stream_interval_ms = int(1000 / self.publish_rate_hz)
        self.streamer = SteveStreamer(self.client, threadsafe=True)
        self.streamer.register_callback(self._stream_callback)
        self.streamer.start_stream(interval_ms=stream_interval_ms)

        self.get_logger().info("Connected to STEVE device")

    def start(self) -> None:
        """Start valve operation."""
        if not self.client:
            self.get_logger().error("Not connected. Call connect() first.")
            return

        try:
            self.client.enable_motor()
            time.sleep(0.5)
            self.client.start_valve()
            self.get_logger().info("Started valve operation")
        except Exception as e:
            self.get_logger().error(f"Failed to start valve: {e}")

    def disconnect(self) -> None:
        """Disconnect from STEVE device."""
        if self.streamer:
            try:
                self.streamer.stop_stream()
            except Exception:
                pass

        if self.client:
            try:
                self.client.stop_valve()
                self.client.disconnect()
            except Exception:
                pass

        self.get_logger().info("Disconnected from STEVE device")

    def _stream_callback(self, sample: Dict[str, Any]) -> None:
        """Callback for streaming data."""
        self._last_sample = sample

    def _publish_callback(self) -> None:
        """Timer callback for publishing."""
        if not self._last_sample or not self._last_sample.get("data_valid"):
            return

        sample = self._last_sample

        # Publish JointState
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["valve_joint"]
        joint_state.position = [sample.get("position_deg", 0) * 3.14159 / 180.0]  # Convert to rad
        joint_state.velocity = [sample.get("omega_rad_s", 0)]
        joint_state.effort = [sample.get("torque_nm", 0)]
        self.state_pub.publish(joint_state)

        # Publish individual values
        position_msg = Float64()
        position_msg.data = joint_state.position[0]
        self.position_pub.publish(position_msg)

        velocity_msg = Float64()
        velocity_msg.data = joint_state.velocity[0]
        self.velocity_pub.publish(velocity_msg)

        torque_msg = Float64()
        torque_msg.data = joint_state.effort[0]
        self.torque_pub.publish(torque_msg)

        # Publish status periodically
        if int(time.time() * 10) % 10 == 0:  # Every second
            try:
                status = self.client.get_status()
                import json

                status_msg = String()
                status_msg.data = json.dumps(status)
                self.status_pub.publish(status_msg)
            except Exception:
                pass

    def _torque_cmd_callback(self, msg: Float64) -> None:
        """Callback for torque commands."""
        if not self.client:
            return

        torque = msg.data

        try:
            self.client.set_torque(torque)
            self.get_logger().debug(f"Set torque: {torque:.3f} N·m")
        except Exception as e:
            self.get_logger().error(f"Failed to set torque: {e}")

    def __del__(self) -> None:
        """Cleanup on deletion."""
        self.disconnect()


def launch_bridge(
    device_ip: str,
    node_name: str = "steve_valve",
    publish_rate_hz: int = 50,
) -> None:
    """
    Launch ROS bridge node (convenience function).
    
    Args:
        device_ip: STEVE device IP address
        node_name: ROS node name
        publish_rate_hz: Publishing rate
    """
    rclpy.init()

    bridge = SteveRosBridge(
        device_ip=device_ip,
        node_name=node_name,
        publish_rate_hz=publish_rate_hz,
    )

    bridge.connect()
    bridge.start()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.disconnect()
        rclpy.shutdown()


# Entry point for ROS2 launch
def main():
    """Main entry point for ROS2 launch."""
    import sys

    if len(sys.argv) < 2:
        print("Usage: ros2 run pysteve steve_bridge <device_ip>")
        sys.exit(1)

    device_ip = sys.argv[1]
    launch_bridge(device_ip)


if __name__ == "__main__":
    main()
