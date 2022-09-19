import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) #twist is data type for angular and linear vel
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.scan_data = []

    def process_scan(self, msg):
        K = 2
        if msg.ranges[45] != 0.0 and msg.ranges[135] != 0:
            error = msg.ranges[45] - msg.ranges[135]
            twt = Twist()
            twt.linear.x = 0.1
            twt.angular.z = error * K # This is the scale for the 
            self.pub.publish(twt)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallFollowerNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()