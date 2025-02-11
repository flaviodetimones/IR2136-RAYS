import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from pymavlink import mavutil
import time
import sys

class MissionControlNode(Node):
    def __init__(self, target_lat, target_lon, target_alt):
        super().__init__('mission_control_node')
        
        # Publisher for target GPS
        self.gps_pub = self.create_publisher(NavSatFix, 'target_gps', 10)
        
        # Subscriber for battery status
        self.battery_sub = self.create_subscription(Float32, 'battery_status', self.check_battery, 10)
        
        # Initialize connection to SITL
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info("Connected to SITL.")
        
        # Assign the input GPS and altitude values
        self.target_lat = target_lat
        self.target_lon = target_lon
        self.target_alt = target_alt
        
        # Publish target GPS
        self.publish_target_gps(self.target_lat, self.target_lon)
        
        # Set the mode to GUIDED
        self.set_mode('GUIDED')
        time.sleep(2)
        
        # Arm the drone and initiate takeoff
        self.arm_vehicle()
        time.sleep(2)
        self.takeoff(self.target_alt)
        time.sleep(2)
        
        # Publish target GPS again after takeoff
        self.publish_target_gps(self.target_lat, self.target_lon)

    def publish_target_gps(self, lat, lon):
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f"Published target GPS: {lat}, {lon}")

    def check_battery(self, msg):
        """Check battery status and land if it is below 20%."""
        if msg.data <= 20.0:
            self.get_logger().warn("Low battery! Initiating landing.")
            self.land_vehicle()

    def arm_vehicle(self):
        """Arms the drone."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm
            0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Drone armed.")

    def takeoff(self, altitude):
        """Initiates takeoff."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, altitude  # Desired altitude
        )
        self.get_logger().info(f"Taking off to {altitude} meters.")

    def set_mode(self, mode):
        """Sets the flight mode for the drone."""
        # Set mode using pymavlink, modes like 'GUIDED', 'STABILIZE', etc.
        self.connection.set_mode(mode)
        self.get_logger().info(f"Mode changed to {mode}.")

    def land_vehicle(self):
        """Sends the landing command to the drone."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Landing command sent.")

def main(args=None):
    """Main function to handle input and run the node."""
    rclpy.init(args=args)
    
    if len(sys.argv) < 4:
        print("Usage: mission_control_node.py <latitude> <longitude> <altitude>")
        return
    
    try:
        lat = float(sys.argv[1])
        lon = float(sys.argv[2])
        alt = float(sys.argv[3])
    except ValueError:
        print("Invalid input. Please enter valid numbers for latitude, longitude, and altitude.")
        return
    
    node = MissionControlNode(lat, lon, alt)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
