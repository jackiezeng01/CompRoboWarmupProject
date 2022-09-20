import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) #twist is data type for angular and linear vel
        self.pub_marker = self.create_publisher(Marker, 'detected_object', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.scan_data = []

    def process_scan(self, msg):
        K = 2
        self.create_marker(msg)
        if msg.ranges[45] != 0.0 and msg.ranges[135] != 0:
            error = msg.ranges[45] - msg.ranges[135]
            twt = Twist()
            twt.linear.x = 0.1
            twt.angular.z = error * K # This is the scale for the 
            self.pub.publish(twt)

    def create_marker(self, msg):
        p1 = Point()
        p1.x = msg.ranges[45] * math.cos(math.pi * (45/180))
        p1.y = msg.ranges[45] * math.sin(math.pi * (45/180))

        p2 = Point()
        p2.y = msg.ranges[135] * math.sin(math.pi * (135/180))
        p2.x = msg.ranges[135] * math.cos(math.pi * (135/180))

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.scale.x = 0.05

        marker.points.append(p1)
        marker.points.append(p2)
        marker.type = Marker.LINE_STRIP

        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = WallFollowerNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()