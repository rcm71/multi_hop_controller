import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Vector3
from drone_msgs.msg import DroneStatus
from sensor_msgs.msg import LaserScan, Image
import random
import time

class DroneNode(Node):

    def __init__(self, drone_id):
        # Initialization setting the drone ID's
        super().__init__("drone_" + str(drone_id))
        self.id = drone_id
        self.bridge = CvBridge()
        self.blob_position = None

        # Initializing the publisher, this is what broadcasts 
        # the variables of the drone itself
        self.publisher = self.create_publisher(
            DroneStatus, 
            "/drone_status", 
            10
        )

        # subscribe to /drone_status
        self.create_subscription(
            DroneStatus,
            "/drone_status",
            self.status_callback,
            10
        )

        # Subscribe to LIDAR
        self.create_subscription(
            LaserScan, 
            "/scan", 
            self.lidar_callback, 
            10
        )

        # Subscribe to camera stream
        self.create_subscription(
            Image, 
            "/camera/image_raw", 
            self.camera_callback, 
            10
        )

        # Runs the collision-checking logic every 0.1 seconds (does NOT publish anything).
        self.collision_timer = self.create_timer(0.1, self.collision_check)

        # publishes (broadcasts) every 0.2 seconds
        self.status_timer = self.create_timer(0.2, self.publish_status)

    # Timer callback: runs every 0.2 seconds.
    # Creates a DroneStatus message containing this drone’s
    # current state (position, velocity, battery, link quality, role, timestamp)
    # and publishes it on the /drone_status topic so other drones can receive it.
    def publish_status(self):
        # Initializing drone node
        msg = DroneStatus()
        msg.id = self.id

        # Example: fake data for simulation for position
        msg.position = Point(
            x=random.uniform(0, 50),
            y=random.uniform(0, 50),
            z=random.uniform(5, 20)
        )

        # Example: fake data for simulation for velocity
        msg.velocity = Vector3(
            x=0.1, y=0.0, z=0.0
        )
        # battery level
        msg.battery_level = random.uniform(30, 100)
        # link quality
        msg.link_quality = random.uniform(0, 1)
        # role
        msg.role = "relay"
        # time
        msg.timestamp = time.time()

        # broadcasts these 
        self.publisher.publish(msg)

    # This method is called whenever ANY DroneStatus message is received.
    #
    # We ignore messages that come from THIS drone (self.id),
    # but print info for all OTHER drones.
    def status_callback(self, msg):
        if msg.id != self.id:
            self.get_logger().info(
                f"Received status from Drone {msg.id}: "
                f"Position: ({msg.position.x:.1f}, {msg.position.y:.1f}, {msg.position.z:.1f}), "
                f"link={msg.link_quality:.2f}"
            )
    
    # Takes the LIDAR scan array (msg.ranges) and stores the minimum distance in front of the drone.
    def lidar_callback(self, msg):
        self.front_distance = min(msg.ranges)

    # The camera feed is used for detecting simple visual obstacles (like a blue object).
    def camera_callback(self, msg):
        self.blob_position = detect_blue_blob(self, msg)

    def collision_check(self):
        if self.front_distance < 1.0:
            self.get_logger().warn("LIDAR obstacle!")

        if self.check_tf_obstacle() < 2.0:
            self.get_logger().warn("Drone nearby!")

        if self.blob_position is not None:
            self.get_logger().info("Blue object detected")

    def detect_blue_blob(self, img_msg):
        # Convert ROS2 Image → OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Convert BGR → HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Blue color range (tweak if needed)
        lower_blue = np.array([100, 120, 70])
        upper_blue = np.array([140, 255, 255])

        # Mask all pixels NOT in blue range
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return None   # No blob detected

        # Largest contour = likely the blob
        largest = max(contours, key=cv2.contourArea)

        # Ignore tiny noise blobs
        if cv2.contourArea(largest) < 200:
            return None

        # Compute centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy)

def main():
    rclpy.init()
    node = DroneNode(drone_id=1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()