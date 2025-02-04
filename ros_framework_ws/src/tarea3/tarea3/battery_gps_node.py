import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pymavlink import mavutil

class BatteryGpsNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)  # Publishes battery status
        self.gps_sub = self.create_subscription(Point, 'target_gps', self.gps_callback, 10)  # Subscribes to target_gps
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # MAVLink connection
        self.connection.wait_heartbeat()  # Wait for first heartbeat to confirm connection
        self.create_timer(1.0, self.timer_callback)  # Publishes battery status every second

    def timer_callback(self):
        # Retrieves battery status
        battery_remaining = self.read_battery_status()
        if battery_remaining is not None:
            msg = Float32()
            msg.data = battery_remaining  # Publish battery status
            self.battery_pub.publish(msg)

        else:
            self.get_logger().warn("Failed to get battery status.")

    def gps_callback(self, msg):
        # Callback to receive GPS position and send to drone
        latitude = msg.x
        longitude = msg.y
        altitude = msg.z
        self.get_logger().info(f"Received GPS target: Lat={latitude}, Lon={longitude}, Alt={altitude}")
        self.go_to_position(latitude, longitude, altitude)

    def read_battery_status(self):
        # Reads battery status from MAVLink
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            return msg.battery_remaining
        return None

    def go_to_position(self, lat, lon, alt):
        # Sends position to the drone via MAVLink
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Reference frame
            0b110111111000,  # Ignore velocity and acceleration
            int(lat * 1e7),  # Latitude as integer
            int(lon * 1e7),  # Longitude as integer
            alt,  # Desired altitude in meters
            0, 0, 0,  # Velocities
            0, 0, 0,  # Accelerations
            0, 0  # Yaw and rate
        )
        self.get_logger().info(f"Sending drone to Lat={lat}, Lon={lon}, Alt={alt}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGpsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
