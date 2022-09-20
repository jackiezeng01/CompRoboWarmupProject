import rclpy
import math
from enum import Enum
from rclpy.node import Node
from .person_follower import PersonFollowerNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

class State(Enum):
    FOLLOW = 1
    CIRCLE = 2
    DANCE = 3

class FiniteStateControllerNode(Node):
    def __init__(self):
        super().__init__('finite_state_controller_node')
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(Marker, 'detected_object', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.max_angle = 45
        self.max_distance = 3
        self.object_distance = 0
        self.object_angle = 0
        self.idle_count = 0
        self.dance_count = 0
        self.dance_speed = 7.0
        self.state = State.CIRCLE
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_scan(self, msg):
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
        
        if (count == 0):
            self.object_distance = 0
            self.object_angle = 0
        else:
            cent_x = total_x/count
            cent_y = total_y/count

            self.object_angle = math.degrees(math.atan(cent_y/cent_x))
            self.object_distance = math.sqrt(cent_x ** 2 + cent_y ** 2)
            marker = self.create_marker(cent_x, cent_y)
            self.pub_marker.publish(marker)

    def create_marker(self, x, y) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
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
        marker.type = Marker.SPHERE
        return marker

    def circle_around(self):
        new_twist = Twist()
        new_twist.angular.z = 1.0
        new_twist.linear.x = 0.5
        self.pub_velocity.publish(new_twist)

        if (self.object_angle == 0 and self.object_distance == 0):
            print("No object")
        else:
            self.state = State.DANCE
        
    def follow_person(self):
        new_twist = Twist()
        new_twist.angular.z = 0.03 * self.object_angle
        new_twist.linear.x = 0.3 * self.object_distance
        self.pub_velocity.publish(new_twist)

        if self.idle_count == 20:
            self.state = State.CIRCLE
            self.idle_count = 0
        elif (self.object_angle == 0 and self.object_distance == 0):
            self.idle_count += 1
        else:
            self.idle_count = 0
    
    def dance_with_joy(self):
        new_twist = Twist()
        if self.dance_count > 27:
            new_twist.angular.z = 0.0
            new_twist.linear.x = 0.0
        else:
            new_twist.angular.z = self.dance_speed
            new_twist.linear.x = 0.0

        self.dance_count += 1
        self.pub_velocity.publish(new_twist)

        if self.dance_count == 35:
            self.dance_count = 0
            self.state = State.FOLLOW
        
    def run_loop(self):
        if self.state == State.FOLLOW:
            self.follow_person()
        elif self.state == State.DANCE:
            self.dance_with_joy()
        elif self.state == State.CIRCLE:
            self.circle_around()

        print(self.state)
    
def main(args=None):
    rclpy.init(args=args)                 # Initialize communication with ROS
    node = FiniteStateControllerNode()      # Create our Node
    rclpy.spin(node)                        # Run the Node until ready to shutdown
    rclpy.shutdown()                        # cleanup

if __name__ == '__main__':
    main()