import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def process_scan(self, msg):
        print(msg.header)
        twt = Twist()
        twt.linear.x = 1.0
        self.pub.publish(twt)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallFollowerNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()