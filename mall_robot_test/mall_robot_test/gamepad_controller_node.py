import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class GamepadController(Node):
    def __init__(self):
        super().__init__('gamepad_controller')
        
        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Publisher for the /cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Scaling factors (adjust based on robot capabilities)
        self.linear_scale = 0.4  # Maximum linear speed in m/s
        self.angular_scale = 1.0  # Maximum angular speed in rad/s
        
        self.get_logger().info('GamepadController node started')

    def joy_callback(self, msg):
        twist_msg = Twist()
        
        # Check if button R1 is pressed
        if msg.buttons[5] == 1:
            # Map joystick axes to velocity commands
            # Axis 1 -> Linear velocity (Forward/Backward)
            # Axis 3 -> Angular velocity (Left/Right)
            twist_msg.linear.x = msg.axes[1] * self.linear_scale
            twist_msg.angular.z = msg.axes[3] * self.angular_scale
        else:
            # If button 13 is not pressed, set velocities to zero
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        # Publish the velocity command
        self.publisher.publish(twist_msg)
        
        self.get_logger().info(f'Published cmd_vel: linear={twist_msg.linear.x:.2f}, angular={twist_msg.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
