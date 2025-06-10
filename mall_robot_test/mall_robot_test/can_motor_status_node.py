import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mall_robot_msgs.msg import RMDMotorStatus
import can
import struct

class CANMotorStatusNode(Node):
    def __init__(self):
        super().__init__('can_motor_status_node')
        self.publisher_ = self.create_publisher(RMDMotorStatus, 'rmd_motor_status', 10)
        self.timer = self.create_timer(0.1, self.read_motor_status)

        # Initialize CAN bus (assuming slcand and can0 are active)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

        # Motor ID and CAN IDs
        self.motor_id = 0x01
        self.tx_id = 0x140 + self.motor_id
        self.rx_id = 0x240 + self.motor_id

    def send_cmd_and_recv(self, cmd_byte):
        msg = can.Message(arbitration_id=self.tx_id, data=[cmd_byte] + [0x00]*7, is_extended_id=False)
        try:
            self.bus.send(msg)
            response = self.bus.recv(timeout=0.1)
            if response and response.arbitration_id == self.rx_id and response.data[0] == cmd_byte:
                return response.data
        except can.CanError as e:
            self.get_logger().warn(f"CAN Error: {e}")
        return None

    def read_motor_status(self):
        status_msg = RMDMotorStatus()
        status_msg.header = Header()
        status_msg.header.stamp = self.get_clock().now().to_msg()

        # 0x9A: Motor status 1 and error flags
        data_9a = self.send_cmd_and_recv(0x9A)
        if data_9a:
            status_msg.motor_temperature = float(data_9a[1])
            status_msg.bus_voltage = struct.unpack('<H', bytes(data_9a[2:4]))[0] * 0.1
            status_msg.bus_current = struct.unpack('<h', bytes(data_9a[4:6]))[0] * 0.1
            status_msg.error_code = data_9a[7]
        else:
            self.get_logger().warn("No response for 0x9A")

        # 0x9C: Phase current
        data_9c = self.send_cmd_and_recv(0x9C)
        if data_9c:
            status_msg.phase_current_a = struct.unpack('<h', bytes(data_9c[1:3]))[0] * 0.01
            status_msg.phase_current_b = struct.unpack('<h', bytes(data_9c[3:5]))[0] * 0.01
            status_msg.phase_current_c = struct.unpack('<h', bytes(data_9c[5:7]))[0] * 0.01
        else:
            self.get_logger().warn("No response for 0x9C")

        # 0x9D: MOSFET temperature
        data_9d = self.send_cmd_and_recv(0x9D)
        if data_9d:
            status_msg.mos_temperature = struct.unpack('<H', bytes(data_9d[1:3]))[0]
        else:
            self.get_logger().warn("No response for 0x9D")

        self.publisher_.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CANMotorStatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
