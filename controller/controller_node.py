import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from drone_msgs.msg import DroneStatus, DroneCommand
import math
import time

class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller")

        # Tracks the last known status of every drone
        self.drone_states = {}   # { id: DroneStatus }

        # Subscribe to all drone states
        self.create_subscription(
            DroneStatus,
            "/drone_status",
            self.status_callback,
            10
        )

        # Publisher to send commands to drones
        self.cmd_pub = self.create_publisher(
            DroneCommand,
            "/drone_commands",
            10
        )

        # Run control loop at 0.2 seconds
        self.timer = self.create_timer(0.2, self.control_loop)

    def status_callback(self, msg):
        """Store latest state for every drone"""
        self.drone_states[msg.id] = msg

    def control_loop(self):
        """Main controller logic: compute where drones should go"""

        if len(self.drone_states) == 0:
            return

        # Example: simple formation target
        # Hard coded for now
        formation_positions = {
            1: Point(x=10.0, y=0.0, z=10.0),
            2: Point(x=10.0, y=10.0, z=10.0),
            3: Point(x=10.0, y=-10.0, z=10.0)
        }

        for drone_id, state in self.drone_states.items():
            target = formation_positions.get(drone_id, None)
            if target is None:
                continue

            cmd = DroneCommand()
            cmd.id = drone_id
            cmd.target_position = target
            cmd.speed = 1.5
            cmd.mode = "goto"

            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f"Sent command to Drone {drone_id}: "
                f"Go to ({target.x}, {target.y}, {target.z})"
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
