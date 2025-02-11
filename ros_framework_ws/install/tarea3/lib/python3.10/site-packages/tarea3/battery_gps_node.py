import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time

class BatteryGPSNode(Node):
    def __init__(self):
        super().__init__('battery_gps_node')
        
        # Publisher for battery status
        self.battery_publisher = self.create_publisher(Int32, 'battery_status', 10)
        # Subscriber for target GPS
        self.target_subscriber = self.create_subscription(
            Point, 'target_gps', self.target_gps_callback, 10)
        
        # Initialize connection to SITL
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL.")
        
        # Start timer to send battery status
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        
    def publish_battery_status(self):
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            remaining = msg.battery_remaining  # Battery percentage
            self.get_logger().info(f"Battery: {remaining}%")
            battery_msg = Int32()
            battery_msg.data = remaining
            self.battery_publisher.publish(battery_msg)
    
    def target_gps_callback(self, msg):
        # Send the received GPS coordinates to the drone
        lat = msg.x
        lon = msg.y
        alt = msg.z
        self.get_logger().info(f"Received target GPS: Lat={lat}, Lon={lon}, Alt={alt}")
        self.send_gps_to_drone(lat, lon, alt)

    def send_gps_to_drone(self, lat, lon, alt):
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame of reference
            0b110111111000,  # Ignore velocity and acceleration
            int(lat * 1e7),  # Latitude as integer
            int(lon * 1e7),  # Longitude as integer
            alt,  # Desired altitude
            0, 0, 0,  # Velocities
            0, 0, 0,  # Accelerations
            0, 0  # Yaw and rate
        )
        self.get_logger().info(f"Moving the drone to Lat={lat}, Lon={lon}, Alt={alt} m")
    

def main(args=None):
    rclpy.init(args=args)
    battery_gps_node = BatteryGPSNode()
    rclpy.spin(battery_gps_node)
    battery_gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
