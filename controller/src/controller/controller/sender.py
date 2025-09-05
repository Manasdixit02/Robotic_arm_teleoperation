import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.subscription = self.create_subscription(
            String,
            'arduino_cmd',
            self.listener_callback,
            10)
        # Update the port below to match your Arduino (e.g., /dev/ttyACM0 or /dev/ttyUSB0)
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    def listener_callback(self, msg):
        self.ser.write((msg.data + '\n').encode())
        self.get_logger().info(f"Sent to Arduino serial: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
