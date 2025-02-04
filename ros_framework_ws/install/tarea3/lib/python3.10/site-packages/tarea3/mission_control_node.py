import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pymavlink import mavutil
import sys
import time

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.gps_pub = self.create_publisher(Point, 'target_gps', 10)  # Publishes the target GPS coordinates
        self.create_timer(1.0, self.monitor_battery)  # Checks the battery status every second
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # MAVLink connection
        self.connection.wait_heartbeat()  # Wait for first heartbeat to confirm connection

        self.lat = None
        self.lon = None
        self.alt = None

    def monitor_battery(self):
        # Monitor battery status and land if battery is under 20%
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            battery_remaining = msg.battery_remaining
            if battery_remaining <= 20:
                self.land_vehicle()

    def land_vehicle(self):
        # Sends the landing command to the drone
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Landing command sent due to low battery.")

    def set_gps_target(self, lat, lon, alt):
        # Set GPS target coordinates
        self.lat = lat
        self.lon = lon
        self.alt = alt

    def send_gps_target(self):
        gps_target = Point()
        gps_target.x = self.lat
        gps_target.y = self.lon
        gps_target.z = self.alt
        self.gps_pub.publish(gps_target)
        self.get_logger().info(f"Sending GPS target: Lat={self.lat}, Lon={self.lon}, Alt={self.alt}")
        self.go_to_position(self.lat, self.lon, self.alt)

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
        self.get_logger().info(f"Moving drone to Lat={lat}, Lon={lon}, Alt={alt}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()

    if len(sys.argv) != 4:
        print("Usage: ros2 run tarea3 mission_control_node <lat> <lon> <alt>")
        sys.exit(1)

    lat = float(sys.argv[1])
    lon = float(sys.argv[2])
    alt = float(sys.argv[3])

    node.set_gps_target(lat, lon, alt)
    node.send_gps_target()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
