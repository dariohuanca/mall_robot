import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class UltrasonicADCReader(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensors_node')

        # I2C bus and ADS initialization
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ads1 = ADS.ADS1115(i2c, address=0x48)
        self.ads2 = ADS.ADS1115(i2c, address=0x49)

        # Set gain (you can adjust depending on expected input voltage)
        self.ads1.gain = 1
        self.ads2.gain = 1

        # Analog channels
        self.channels = [
            AnalogIn(self.ads1, ADS.P0),
            AnalogIn(self.ads1, ADS.P1),
            AnalogIn(self.ads1, ADS.P2),
            AnalogIn(self.ads1, ADS.P3),
            AnalogIn(self.ads2, ADS.P0),
            AnalogIn(self.ads2, ADS.P1),
            AnalogIn(self.ads2, ADS.P2),
            AnalogIn(self.ads2, ADS.P3),
        ]

        # Create 8 publishers (one per sensor)
        self.publishers = [
            self.create_publisher(Range, f'/ultrasonic_sensor/{i}', 10)
            for i in range(8)
        ]

        # Timer for 10 Hz readout
        self.timer = self.create_timer(0.1, self.publish_ranges)

    def publish_ranges(self):
        try:
            for i, ch in enumerate(self.channels):
                voltage = ch.voltage

                # Example: Assuming 10mV/cm (e.g., MaxBotix analog sensors)
                # Adjust scaling as per your specific sensor model
                distance_cm = voltage * 100.0
                distance_m = distance_cm / 100.0

                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f'ultrasonic_{i}'
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 0.26  # radians, adjust if known
                msg.min_range = 0.02      # meters
                msg.max_range = 5.00      # meters
                msg.range = max(min(distance_m, msg.max_range), msg.min_range)

                self.publishers[i].publish(msg)

        except Exception as e:
            self.get_logger().error(f'ADC read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicADCReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
