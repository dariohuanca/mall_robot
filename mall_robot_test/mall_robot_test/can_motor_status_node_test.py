#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from mall_robot_msgs.msg import RMDMotorStatus

import math
import myactuator_rmd.myactuator_rmd_py as rmd  # Python binding of your C++ SDK

# Constants
WHEEL_RADIUS = 0.1       # in meters
WHEEL_SEPARATION = 0.564 # in meters

class CANMotorStatusNode(Node):
    def __init__(self):
        super().__init__('can_motor_status_node')

        self.driver = rmd.CanDriver("can0")  # or "slcan0" if using slcan
        self.motor_ids = [0x141, 0x142]
        self.motors_position = {0x141: 'right', 0x142: 'left'}

        self.motors = {
            mid: rmd.ActuatorInterface(self.driver, mid - 0x140)  # Adjust to logical ID if needed
            for mid in self.motor_ids
        }

        self.motor_publishers = {
            mid: self.create_publisher(RMDMotorStatus, f'/rmd_motor_status_{self.motors_position[mid]}', 10)
            for mid in self.motor_ids
        }
        self.vel_pub = self.create_publisher(TwistStamped, '/encoder_vel', 10)
        self.latest_vel_ang = {mid: 0.0 for mid in self.motor_ids}

        self.timer = self.create_timer(0.1, self.read_all_motor_status)

    def read_all_motor_status(self):
        for motor_id, motor in self.motors.items():
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
                msg_out.error_flag = status1.error_code

                msg_out.iq_current = status2.current
                msg_out.speed_shaft = status2.shaft_speed
                msg_out.degree_shaft = status2.shaft_angle
                self.latest_vel_ang[motor_id] = status2.haft_speed * math.pi / 180

                msg_out.phase_current_a = status3.current_phase_a
                msg_out.phase_current_b = status3.current_phase_b
                msg_out.phase_current_c = status3.current_phase_c

                self.motor_publishers[motor_id].publish(msg_out)
                self.get_logger().info(f"Status motor {motor_id}: {msg_out}")

            except Exception as e:
                self.get_logger().error(f"Failed reading motor {motor_id}: {e}")

        # Compute and publish robot velocity
        if all(mid in self.latest_vel_ang for mid in self.motor_ids):
            vel_ang_l = self.latest_vel_ang[0x142] * WHEEL_RADIUS
            vel_ang_r = -self.latest_vel_ang[0x141] * WHEEL_RADIUS

            vx = (vel_ang_r + vel_ang_l) / 2.0
            vy = 0.0
            vtheta = (vel_ang_r - vel_ang_l) / WHEEL_SEPARATION

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = "base_link"
            twist_msg.twist.linear.x = vx
            twist_msg.twist.linear.y = vy
            twist_msg.twist.angular.z = vtheta

            self.vel_pub.publish(twist_msg)

    def destroy_node(self):
        try:
            self.get_logger().info("Shutting down actuator interface.")
            self.driver.close()  # if your CanDriver supports explicit shutdown
        except Exception as e:
            self.get_logger().warn(f"Error closing CAN driver: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CANMotorStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()