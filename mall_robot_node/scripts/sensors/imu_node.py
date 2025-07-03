import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055
import time
import math

class BNO055IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # I2C bus init
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = adafruit_bno055.BNO055_I2C(i2c)

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("BNO055 IMU node started")

    def timer_callback(self):
        try:
            # Get orientation (Euler angles in degrees)
            euler = self.bno.euler
            accel = self.bno.acceleration
            gyro = self.bno.gyro

            if euler is None or accel is None or gyro is None:
                self.get_logger().warning("Sensor data not ready")
                return

            msg = Imu()

            # Orientation: convert Euler to quaternion
            heading, roll, pitch = [math.radians(v if v is not None else 0.0) for v in euler]
            q = self.euler_to_quaternion(roll, pitch, heading)
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

            # Angular velocity (rad/s)
            msg.angular_velocity.x = math.radians(gyro[0])
            msg.angular_velocity.y = math.radians(gyro[1])
            msg.angular_velocity.z = math.radians(gyro[2])

            # Linear acceleration (m/s^2)
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Exception in timer callback: {e}")

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles (in radians) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = BNO055IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
