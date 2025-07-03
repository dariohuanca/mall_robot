#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistStamped
from mall_robot_msgs.msg import RMDMotorStatus
import myactuator_rmd.myactuator_rmd_py as rmd
import math

# Constants
WHEEL_RADIUS = 0.127     # in meters
WHEEL_SEPARATION = 0.59  # in meters

class CANMotorControlNode(Node):
    def __init__(self):
        super().__init__('can_motor_control_node')

        # Initialize CAN driver and actuators
        self.driver = rmd.CanDriver("can0")
        self.motor_right = rmd.ActuatorInterface(self.driver, 1)   # CAN ID 0x141
        self.motor_left = rmd.ActuatorInterface(self.driver, 2)  # CAN ID 0x142

        # Publishers and Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.encoder_pub = self.create_publisher(TwistStamped, '/encoder_vel', 10)

        self.motor_publishers = {
            0x141: self.create_publisher(RMDMotorStatus, '/rmd_motor_status_right', 10),
            0x142: self.create_publisher(RMDMotorStatus, '/rmd_motor_status_left', 10)
        }

        self.latest_vel_ang = {0x141: 0.0, 0x142: 0.0}

        # Timer for motor status polling
        self.timer = self.create_timer(0.1, self.read_all_motor_status)

        self.get_logger().info("can_motor_control_node started using myactuator_rmd_py")

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Compute desired motor speeds (deg/s)
        v_right_dps = (180 / math.pi) * (linear_x + (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS
        v_left_dps = (180 / math.pi) * (linear_x - (WHEEL_SEPARATION / 2.0) * angular_z) / WHEEL_RADIUS

        try:
            vel_enc_r = self.motor_right.sendVelocitySetpoint(-v_right_dps)
            vel_enc_l = self.motor_left.sendVelocitySetpoint(v_left_dps)

            self.get_logger().info(
                f"Cmd -> R: {-v_right_dps:.2f} dps, L: {v_left_dps:.2f} dps"
            )

        except rmd.can.SocketException as e:
            self.get_logger().error(f"CAN error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def read_all_motor_status(self):
        motors = {0x141: self.motor_right, 0x142: self.motor_left}

        for motor_id, motor in motors.items():
            msg_out = RMDMotorStatus()
            msg_out.header = Header()
            msg_out.header.stamp = self.get_clock().now().to_msg()

            try:
                status1 = motor.getMotorStatus1()
                status2 = motor.getMotorStatus2()
                status3 = motor.getMotorStatus3()

                msg_out.temperature = status1.temperature
                msg_out.brake_control_command = status1.is_brake_released
                msg_out.bus_voltage = status1.voltage
                msg_out.error_flag = status1.error_code.name

                msg_out.iq_current = status2.current
                msg_out.speed_shaft = status2.shaft_speed
                msg_out.degree_shaft = status2.shaft_angle
                self.latest_vel_ang[motor_id] = status2.shaft_speed * math.pi / 180.0

                msg_out.phase_current_a = status3.current_phase_a
                msg_out.phase_current_b = status3.current_phase_b
                msg_out.phase_current_c = status3.current_phase_c

                self.motor_publishers[motor_id].publish(msg_out)

            except Exception as e:
                self.get_logger().error(f"Failed reading motor {motor_id}: {e}")

        # Publish computed robot velocity
        if all(mid in self.latest_vel_ang for mid in motors):
            vel_ang_r = -self.latest_vel_ang[0x141] * WHEEL_RADIUS
            vel_ang_l = self.latest_vel_ang[0x142] * WHEEL_RADIUS

            vx = (vel_ang_r + vel_ang_l) / 2.0
            vtheta = (vel_ang_r - vel_ang_l) / WHEEL_SEPARATION

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.linear.x = vx
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.angular.z = vtheta

            self.encoder_pub.publish(twist_msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down actuators...")
        try:
            self.motor_left.shutdownMotor()
            self.motor_right.shutdownMotor()
            self.driver.close()
        except Exception as e:
            self.get_logger().warn(f"Error during shutdown: {e}")
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
