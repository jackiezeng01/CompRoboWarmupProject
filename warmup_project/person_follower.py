from curses import COLOR_CYAN
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, 'detected_object', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.max_angle = 45
        self.max_distance = 3
        self.object_distance = 0
        self.object_angle = 0
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_scan(self, msg):
        total_x = 0.0
        total_y = 0.0
        count = 0
        for i in range(0,360):
            distance = msg.ranges[i]
            if (((i >= 0 and i <= self.max_angle) or (360 - self.max_angle <= i and i < 360)) and (distance != 0 and distance < self.max_distance)):
                x = distance * math.sin(math.pi * (i/180))
                y = distance * math.cos(math.pi * (i/180))

                total_x += x
                total_y += y

                count += 1
        
        if (count == 0):
            self.object_distance = 0
            self.object_angle = 0
        else:
            cent_x = total_x/count
            cent_y = total_y/count

            self.object_angle = math.degrees(math.atan(cent_x/cent_y))
            self.object_distance = math.sqrt(cent_x ** 2 + cent_y ** 2)
            #print(cent_x, cent_y)
            #print(self.object_angle, self.object_distance)
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.pose.position.x = cent_y
            marker.pose.position.y = cent_x
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.type = Marker.SPHERE
            self.pub_marker.publish(marker)

    def run_loop(self):
        new_twist = Twist()
        new_twist.angular.z = 0.03 * self.object_angle
        new_twist.linear.x = 0.3 * self.object_distance
        self.pub_velocity.publish(new_twist)
            
        

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = PersonFollowerNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()