import math
import numpy as np
from struct import calcsize
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(MarkerArray, 'detected_object', 10)
        self.sub = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        # self.sub = self.create_subscription(Odometry, 'odom', self.)
        self.dest = [-20,0]
        self.max_angle = 90
        self.max_distance = 3
        self.timer = self.create_timer(0.1, self.run_loop)
        self.drive_angle = 0
        self.drive_distance = 0

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
                # markers.markers.append(self.create_marker_advanced(cent_x,cent_y, 0, 0, len(object_list), 0))

                total_x = 0.0
                total_y = 0.0
                count = 0
        
        print(object_list)
        self.drive_angle, self.drive_distance = self.calculate_direction(object_list)
        print("drive angle and distance: ",self.drive_angle, self.drive_distance)
        self.pub_marker.publish(markers)

    def cart_to_polar(self, x, y):
        r = np.sqrt(x**2 + y**2)
        degrees = math.degrees(np.arctan2(y, x))
        return(r, degrees)

    def process_obstacle_vector(self, x, y):
        # given a point of the obstacle, calculate the unit vector from it to the robot
        mag = math.sqrt(x**2 + y**2)
        unit_vector = [x/mag, y/mag]
        # intensity of opposition vector linearly scaled based on how close it is to the robot
        scaled_mag = (-mag+3)
        return unit_vector, scaled_mag

    def calculate_direction(self, vector_list):
        # calculate direction the robot should go by adding up the negative vectors and the goal destination. 
        x_sum, y_sum = self.dest[0], self.dest[1]
        for vector in vector_list:
            unit_vector, scaled_mag = self.process_obstacle_vector(vector[0], vector[1])
            print ("unit_vector: ", unit_vector, "scaled_mag", scaled_mag)
            x_sum -= scaled_mag*unit_vector[0]
            y_sum -= scaled_mag*unit_vector[1] 
        # turn vectors into polar
        print ("xsum and ysum: ", x_sum, y_sum)
        return self.cart_to_polar(x_sum, y_sum)

    def run_loop(self):
        pass
        new_twist = Twist()
        new_twist.angular.z = 0.03 * self.drive_angle
        new_twist.linear.x = 0.003 * self.drive_distance
        print(new_twist.angular.z, new_twist.linear.x)
        self.pub_velocity.publish(new_twist)

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
        marker.type = Marker.SPHERE
        return marker

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ObstacleAvoiderNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()