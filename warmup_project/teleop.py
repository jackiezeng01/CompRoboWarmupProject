import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.process_key()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key

    def process_key(self):
        while self.key != '\x03':
            self.key = self.getKey()
            print("Key: " + self.key)
            if self.key == 'w':
                twt = Twist()
                twt.linear.x = 1.0
                self.pub.publish(twt)
            if self.key == 's':
                twt = Twist()
                twt.linear.x = -1.0
                self.pub.publish(twt)
        
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = TeleopNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()