from cmath import polar
from locale import YESEXPR
from xml.sax.handler import property_declaration_handler
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10) #twist is data type for angular and linear vel
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)      
        # goal destination in odom and baselink
        self.dest.x = 5
        self.dest.y = 0
        # If the obstacle is within this distance, we want to leave
        self.max_distance = 3
        # Angle range on each side
        self.max_angle = 90
        # set it to drive straight
        self.direction = 

    def process_scan(self, msg):
        K = 1
        # detect an obstacle
        for i in range(0,360):
            distance = msg.ranges[i]
            # filter for the laser points that are really close to us
            if (((i >= 0 and i <= self.max_angle) or (360 - self.max_angle <= i and i < 360)) and (distance != 0 and distance < self.max_distance)):
                x,y = self.polar_to_cart(distance, i)

            twt = Twist()
            twt.linear.x = 0.1
            twt.angular.z = error * K # This is the scale for the 
            self.pub.publish(twt)

    def polar_to_cart(r,degree):
        x = r * math.sin(math.pi * (degree/180))
        y = r * math.cos(math.pi * (degree/180))
        return [x,y]
    
    def cart_to_polar(x,y):
        r = math.sqrt(x ** 2 + y ** 2)
        degree = math.degrees(math.atan(x/y))
        return [r,degree]

    def negative_intensity(d):
        # alternatively have an equation: maybe intensity = 1/d 
        if d > 0 and d <= 1:
            return 3
        if d > 1 and d <= 2:
            return 2
        if d > 2 and d <= 3:
            return 1
        return 1/d
        return 0

        def create_marker_advanced(self, x, y, dir_x, dir_y, id, type) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = type
        marker.id = id
        # set color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        # set pose and orientation
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = float(dir_x)
        marker.pose.orientation.y = float(dir_y)
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # set scale of the marker
        marker.scale.x = 10.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        return marker

    
    def calculate_direction(self, vector_list):
        # calculate direction the robot should go by adding up the negative vectors and the goal destination. 
        x_sum = self.dest.x
        y_sum = self.dest.y
        for vector in vector_list:
            x_sum + vector[0]
            y_sum + vector[1]
        # turn vectors into polar
        return self.cart_to_polar(x_sum, y_sum)



    
    


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ObstacleAvoiderNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()