import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

class ObstableAvoiderNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(MarkerArray, 'detected_object', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.max_angle = 90
        self.max_distance = 3
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_scan(self, msg):
        object_list = []
        markers = MarkerArray()
        total_x = 0.0
        total_y = 0.0
        count = 0
        for i in range(0,360):
            distance = msg.ranges[i]
            if (((i >= 0 and i <= self.max_angle) or (360 - self.max_angle <= i and i < 360)) and (distance != 0 and distance < self.max_distance)):
                y = distance * math.sin(math.pi * (i/180))
                x = distance * math.cos(math.pi * (i/180))

                total_x += x
                total_y += y

                count += 1
            elif count != 0:
                cent_x = total_x/count
                cent_y = total_y/count

                object_list.append((cent_x, cent_y))

                markers.markers.append(self.create_marker(cent_x,cent_y, len(object_list)))

                total_x = 0.0
                total_y = 0.0
                count = 0
        
        print(object_list)
        self.pub_marker.publish(markers)
        

    def run_loop(self):
        pass

    def create_marker(self, x, y, id) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = id
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.type = Marker.ARROW
        return marker


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ObstableAvoiderNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()