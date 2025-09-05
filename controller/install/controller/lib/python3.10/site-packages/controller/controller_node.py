#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from . import sender  # Import the send_command function

class ControllerNode(Node):
    def __init__(self):
        super().__init__('command_subscriber')

        # Subscribers
        self.create_subscription(Int32, '/cmd_topic_prismatic', self.prismatic_cmd_callback, 10)
        self.create_subscription(Int32, '/cmd_topic_elb_rev', self.elb_rev_cmd_callback, 10)

        # Store the previous command to prevent re-sending the same command
        self.last_prismatic_cmd = None
        self.last_elbow_cmd = None

    def prismatic_cmd_callback(self, msg):
        if msg.data != self.last_prismatic_cmd:
            if msg.data < 0:
                direction = 0
                n_steps = -msg.data
            else:
                direction = 1
                n_steps = msg.data

            # Send the command to the controller via serial
            sender.send_command(direction, n_steps)

            # Update the last command sent
            self.last_prismatic_cmd = msg.data

            # Log it
            self.get_logger().info(f'Prismatic command: direction={direction}, steps={n_steps}')

    def elb_rev_cmd_callback(self, msg):
        if msg.data != self.last_elbow_cmd:
            if msg.data < 0:
                direction = 0
                n_steps = -msg.data
            else:
                direction = 1
                n_steps = msg.data

            # Send the command to the controller via serial
            sender.send_command(direction, n_steps)

            # Update the last command sent
            self.last_elbow_cmd = msg.data

            # Log it
            self.get_logger().info(f'Elbow command: direction={direction}, steps={n_steps}')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

