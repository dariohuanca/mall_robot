import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct

#Constants
# Wheel radius and separation (adjust according to the robot)
WHEEL_RADIUS = 0.02  # in meters
WHEEL_SEPARATION = 0.564  # in meters

class CmdVelToCAN(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_can')
        
        # Configure the CAN interface
        self.can_interface = '/dev/ttyACM0'  # Adjust as needed
        self.bitrate = 1000000  # 1 Mbps
        
        # Initialize the CAN bus
        self.bus = can.interface.Bus(bustype='slcan', channel=self.can_interface, bitrate=self.bitrate)
        
        # Subscribe to the /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Motor IDs
        self.motor_id_left = 0x141
        self.motor_id_right = 0x142

        self.get_logger().info('cmd_vel_to_can node started')

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate each wheel's velocity
        v_right_dps = (linear_x + (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS
        v_left_dps = (linear_x - (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS
        
        # Convert to int32_t format (scale by 100 to match 0.01 dps/LSB)
        v_right_int = int(v_right_dps * 100)
        v_left_int = -int(v_left_dps * 100)

        # Pack as signed int32 (4 bytes) in little-endian format
        data_right = [0xA2] + [0x00, 0x00, 0x00] + list(struct.pack('<i', v_right_int))
        data_left = [0xA2] + [0x00, 0x00, 0x00]+ list(struct.pack('<i', v_left_int))

        # Send CAN commands
        self.send_can_message(self.motor_id_right, data_right)
        self.send_can_message(self.motor_id_left, data_left)
        
        self.get_logger().info(f'Sent: v_left={-v_left_dps:.2f} dps, v_right={v_right_dps:.2f} dps')

    def send_can_message(self, arbitration_id, data):
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        print(message)
        try:
            self.bus.send(message)
            self.get_logger().info(f'CAN message sent to ID {arbitration_id}')
        except can.CanError:
            self.get_logger().error('Error sending CAN message')

    def shutdown(self):
        self.bus.shutdown()
        self.get_logger().info('CAN bus closed')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToCAN()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
