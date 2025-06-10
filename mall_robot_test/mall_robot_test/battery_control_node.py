import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import board
import busio
import adafruit_ina260

class BatteryArrayPublisher(Node):
    def __init__(self):
        super().__init__('battery_control_node')

        # I2C and INA260 Init
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina260 = adafruit_ina260.INA260(i2c)

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, 'battery_control_data', 10)

        # Timer: 1 Hz
        self.timer = self.create_timer(1.0, self.publish_battery_data)

    def publish_battery_data(self):
        try:
            voltage = self.ina260.voltage
            current = self.ina260.current / 1000.0  # Convert mA to A

            msg = Float32MultiArray()
            msg.data = [voltage, current]

            self.pub.publish(msg)
            self.get_logger().info(f"Published: Voltage = {voltage:.2f} V | Current = {current:.3f} A")
        except Exception as e:
            self.get_logger().error(f"INA260 read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryArrayPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
