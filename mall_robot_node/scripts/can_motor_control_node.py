#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import myactuator_rmd.myactuator_rmd_py as rmd

# Constants
WHEEL_RADIUS = 0.1  # meters
WHEEL_SEPARATION = 0.564  # meters

class CANMotorControlNode(Node):
    def __init__(self):
        super().__init__('can_motor_control_node')

        # Initialize the driver and actuators
        self.driver = rmd.CanDriver("can0")
        self.motor_left = rmd.ActuatorInterface(self.driver, 1)   # CAN ID 0x141
        self.motor_right = rmd.ActuatorInterface(self.driver, 2)  # CAN ID 0x142

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info("can_motor_control_node started using myactuator_rmd_py")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute angular velocity in deg/s
        v_right_dps = (180 / 3.14) * (linear_x + (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS
        v_left_dps = (180 / 3.14) * (linear_x - (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS

        # Send speed setpoints to motors (in deg/s, acceleration in deg/s²)
        try:
            status_r = self.motor_right.sendSpeedSetpoint(v_right_dps, 500.0)
            status_l = self.motor_left.sendSpeedSetpoint(-v_left_dps, 500.0)

            self.get_logger().info(
                f"Cmd sent | Left: {v_left_dps:.2f} dps | Right: {v_right_dps:.2f} dps"
            )
        except rmd.can.SocketException as e:
            self.get_logger().error(f"CAN error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down actuators...")
        try:
            self.motor_left.shutdownMotor()
            self.motor_right.shutdownMotor()
        except Exception as e:
            self.get_logger().warn(f"Error during motor shutdown: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANMotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()