#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import myactuator_rmd.myactuator_rmd_py as rmd
import math

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

        self.encoder_pub = self.create_publisher(
            TwistStamped,
            '/encoder_vel',
            10
        )

        self.get_logger().info("can_motor_control_node started using myactuator_rmd_py")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute angular velocity in deg/s
        v_right_dps = (180 / math.pi) * (linear_x + (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS
        v_left_dps = (180 / math.pi) * (linear_x - (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS

        try:
            # Send commands
            vel_enc_r = self.motor_right.sendVelocitySetpoint(v_right_dps).shaft_speed
            vel_enc_l = self.motor_left.sendVelocitySetpoint(-v_left_dps).shaft_speed

            # Convert to rad/s
            w_r = (vel_enc_r * math.pi) / 180.0
            w_l = (vel_enc_l * math.pi) / 180.0

            # Compute robot velocities
            v = (WHEEL_RADIUS / 2.0) * (w_r + w_l)
            w = (WHEEL_RADIUS / WHEEL_SEPARATION) * (w_r - w_l)

            # Publish TwistStampedimport myactuator_rmd.myactuator_rmd_py as rmd
            encoder_msg = TwistStamped()
            encoder_msg.header.stamp = self.get_clock().now().to_msg()
            encoder_msg.twist.linear.x = v
            encoder_msg.twist.angular.z = w
            self.encoder_pub.publish(encoder_msg)

            self.get_logger().info(
                f"Motor cmd -> Right: {v_right_dps:.2f} dps | Left: {-v_left_dps:.2f} dps | "
                f"Robot vel -> Linear: {v:.2f} m/s | Angular: {w:.2f} rad/s"
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