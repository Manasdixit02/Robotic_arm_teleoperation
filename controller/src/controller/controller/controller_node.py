#!/usr/bin/env python3

# NOTE: This node publishes commands to the 'arduino_cmd' topic. To send these commands to your Arduino, you need a serial bridge (e.g., a Python script using pyserial) that subscribes to 'arduino_cmd' and writes the data to the Arduino's serial port. Without this bridge, the RX LED on the Arduino will not blink.

# Keyboard teleop for 5 stepper motors and 2 relays
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

try:
    import termios
    import tty
except ImportError:
    print("This script only works on Unix-like systems.")
    sys.exit(1)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(String, 'arduino_cmd', 10)
        self.selected_motor = 1  # 1, 2, 3, 4, or 5
        self.speed = 50  # percent
        self.relays = {1: False, 2: False}
        self.get_logger().info('Teleop node started. Use keys to control motors and relays.')
        self.print_instructions()

    def print_instructions(self):
        print("""
Keyboard Teleop:
  1/2/3/4/5 : Select motor 1/2/3/4/5
  Arrow Up/Down : Move selected motor +10/-10 degrees
  + / - : Increase/decrease speed by 10%
  r : Toggle relay 1
  t : Toggle relay 2
  q : Quit
        """)

    def send_motor_command(self, motor, direction):
        # direction: +10 or -10
        cmd = f"M{motor}:{direction}"
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent to Arduino: {cmd}")

    def send_speed_command(self, delta):
        # delta: +10 or -10
        cmd = f"SPEED:{delta}"
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent to Arduino: {cmd}")

    def send_relay_command(self, relay, state):
        cmd = f"R{relay}:{'ON' if state else 'OFF'}"
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent to Arduino: {cmd}")

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == '1':
                self.selected_motor = 1
                print("Selected motor 1")
            elif key == '2':
                self.selected_motor = 2
                print("Selected motor 2")
            elif key == '3':
                self.selected_motor = 3
                print("Selected motor 3")
            elif key == '4':
                self.selected_motor = 4
                print("Selected motor 4")
            elif key == '5':
                self.selected_motor = 5
                print("Selected motor 5")
            elif key == '+':
                self.send_speed_command('+10')
                print("Speed increased by 10%")
            elif key == '-':
                self.send_speed_command('-10')
                print("Speed decreased by 10%")
            elif key == 'r':
                self.relays[1] = not self.relays[1]
                self.send_relay_command(1, self.relays[1])
            elif key == 't':
                self.relays[2] = not self.relays[2]
                self.send_relay_command(2, self.relays[2])
            elif ord(key) == 27:  # Arrow keys
                next1 = get_key()
                next2 = get_key()
                if next1 == '[':
                    if next2 == 'A':  # Up
                        self.send_motor_command(self.selected_motor, '+10')
                    elif next2 == 'B':  # Down
                        self.send_motor_command(self.selected_motor, '-10')
            elif key == 'q':
                print("Exiting teleop.")
                break
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

