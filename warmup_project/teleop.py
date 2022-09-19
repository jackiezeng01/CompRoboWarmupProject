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
        self.linSpeed = 0.5
        self.angSpeed = 1.0
        self.process_key()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def process_key(self):
        while self.key != '\x03':
            self.key = self.getKey()
            print("Key: " + self.key)
            twt = Twist()
            if self.key == 'w':
                twt.linear.x = self.linSpeed
                twt.angular.z = 0.0
            elif self.key == 's':
                twt.linear.x = -self.linSpeed
                twt.angular.z = 0.0
            elif self.key == 'a':
                twt.linear.x = self.linSpeed/3
                twt.angular.z = self.angSpeed
            elif self.key == 'd':
                twt.linear.x = self.linSpeed/3
                twt.angular.z = -self.angSpeed
                
            self.pub.publish(twt)
        
def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = TeleopNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()