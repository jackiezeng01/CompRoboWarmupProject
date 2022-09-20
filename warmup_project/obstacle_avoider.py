import math
from ssl import get_default_verify_paths
import numpy as np
from struct import calcsize
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

class Vector2D():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, v2):
        x_sum = self.x + v2.x
        y_sum = self.y + v2.y
        return Vector2D(x_sum, y_sum)

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y        

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.pub_velocity = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_marker = self.create_publisher(MarkerArray, 'detected_object', 10)
        self.sub_laser = self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.process_odom, 10)

        self.goal = Point(5,0)
        self.goal_vector_weight_x = 0.5
        self.goal_vector_weight_y = 1
        self.max_angle = 90
        self.max_distance = 3
        self.direction = Vector2D(0,0)

        self.timer = self.create_timer(0.1, self.run_loop)
        self.drive_angle = 0
        self.drive_distance = 0

        self.robot_pose = Point(0,0)
        
    def process_odom(self, msg):
        '''
        sending where the robot's location is, and its orientation. 
        '''
        _, _, self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        goal_vector_x = - (self.goal.y - msg.pose.pose.position.y) # flipped bc of 
        goal_vector_y = self.goal.x - msg.pose.pose.position.x
        self.robot_pose = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.goal_vector = Vector2D(goal_vector_x*self.goal_vector_weight_x, goal_vector_y*self.goal_vector_weight_y)

    def process_scan(self, msg):
        obstacle_vector_list = []
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
                obstacle_vector = Vector2D(cent_x, cent_y)

                obstacle_vector_list.append(obstacle_vector)
                markers.markers.append(self.create_marker(cent_x,cent_y, len(obstacle_vector_list)))

                total_x = 0.0
                total_y = 0.0
                count = 0
            # markers.markers.append(self.create_marker(self.goal.x, self.goal.y, len(obstacle_vector_list))+1)
        
        self.print_obstacle_vectors(obstacle_vector_list)
        self.pub_marker.publish(markers)

        self.direction = self.calculate_direction(obstacle_vector_list)

    def cart_to_polar(self, x, y):
        r = math.sqrt(x ** 2 + y ** 2)
        degrees = math.degrees(math.atan2(y/x))
        return(r, degrees)
    
    def print_obstacle_vectors(self, obstacle_vector_list):
        list = []
        for vector in obstacle_vector_list:
            list.append([vector.x, vector.y])
        print ("obstacle vectors:", list)

    def process_obstacle_vector(self, x, y):
        # given a point of the obstacle, calculate the unit vector from it to the robot
        mag = math.sqrt(x**2 + y**2)
        unit_vector = [x/mag, y/mag]
        # intensity of opposition vector linearly scaled based on how close it is to the robot
        scaled_mag = (-mag+3)
        return unit_vector, scaled_mag

    def calculate_direction(self, obstacle_vector_list):
        # calculate direction the robot should go by adding up the negative vectors and the goal destination. 
        self.print_obstacle_vectors()
        direction = Vector2D(self.goal_vector.x, self.goal_vector.y)
        for vector in obstacle_vector_list:
            direction = direction.add(vector)
        print("updating direction")
        return direction

    def run_loop(self):
        new_twist = Twist()
        print("direction:", [self.direction.x, self.direction.y])
        angle = math.atan2(self.direction.y, self.direction.x) # in radians
        if abs(get_distance(self.goal, self.robot_pose)) > 0.5:
            new_twist.linear.x = 0.1
            if (self.direction.x < 0 and abs(self.direction.x + self.yaw) < 1):
                new_twist.angular.z = -0.1
            elif (self.direction.x > 0 and abs(self.direction.x - self.yaw) < 1):
                new_twist.angular.z = 0.1
        # print(new_twist.angular.z, new_twist.linear.x)
        self.pub_velocity.publish(new_twist)

    def create_marker(self, x, y, id) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = id
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.type = Marker.SPHERE
        return marker

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def get_distance(pt1, pt2):
    return math.dist([pt1.x, pt1.y], [pt2.x, pt2.y])

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ObstacleAvoiderNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()