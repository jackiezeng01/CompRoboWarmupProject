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
    '''
    Class to manipulate 2D vectors
    '''
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def subtract(self, v2):
        x_sum = self.x - v2.x
        y_sum = self.y - v2.y
        return Vector2D(x_sum, y_sum)
    
    def scale_weight(self, x_weight, y_weight):
        self.x = self.x*x_weight
        self.y = self.y*y_weight

    def __str__(self):
        return "[" + str(self.x) + "," + str(self.y) + "]"

class Point():
    '''
    Class to store cartesian coordinates
    '''
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

        # Laser scan settings
        self.max_angle = 90   # Only scan the front 180 degrees
        self.max_distance = 3   # Only count points that are closer than 3 meters
        
        # Initialize goal destination in cartesian coordinates
        self.goal = Point(5,0)
        self.goal_vector = Vector2D(0,0)
        self.goal_vector_weight_x = 0.5
        self.goal_vector_weight_y = 1

        # These update with every timestep
        self.heading = Vector2D(0,0) # Heading of the robot movement 
        self.robot_pose = Point(0,0) # Current location of the robot
        self.yaw = 0

        # Run loop
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_odom(self, msg):
        '''
        Get information on the robot pose and orientation. 
        Update goal vector based on the robot pose.
        '''
        _, _, self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_pose = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
        goal_vector_x = - (self.goal.y - self.robot_pose.y)
        goal_vector_y = self.goal.x - self.robot_pose.x
        self.goal_vector = Vector2D(goal_vector_x*self.goal_vector_weight_x, goal_vector_y*self.goal_vector_weight_y)
        # print("goal vector: ", self.goal_vector.x, self.goal_vector.y)
    
    def process_scan(self, msg):
        '''
        Processes laser scan data. Identifies obstacle clusters and add the 
        respective opposition vectors to an array.
        '''
        obstacle_vector_list = []
        markers = MarkerArray()
        total_x = 0.0
        total_y = 0.0
        count = 0
        for i in range(0,360):
            distance = msg.ranges[i]
            # Filter for laser points and cluster them
            if (((i >= 0 and i <= self.max_angle) or (360 - self.max_angle <= i and i < 360)) and (distance != 0 and distance < self.max_distance)):

                x,y = polar2cart(i, distance)

                total_x += x
                total_y += y
                count += 1

            elif count != 0:

                # Calculate centroid of the obstacle
                cent_x = total_x/count
                cent_y = total_y/count
                # Create obstacle vector and add to list
                obstacle_vector = Vector2D(cent_x, cent_y)
                obstacle_vector_list.append(obstacle_vector)
                # Create obstacle marker
                markers.markers.append(self.create_marker(cent_x,cent_y, len(obstacle_vector_list)))
                # Reset cluster counter
                total_x = 0.0
                total_y = 0.0
                count = 0
        
        # self.print_obstacle_vectors(obstacle_vector_list)
        self.pub_marker.publish(markers)
        # Update direction the robot should head in
        self.heading = self.calculate_heading(obstacle_vector_list)

    def calculate_heading(self, obstacle_vector_list):
        '''
        Calculate the direction the robot should go by adding up opposing vectors
        and the goal destination vector
        '''
        self.print_obstacle_vectors(obstacle_vector_list)
        heading = self.goal_vector
        for vector in obstacle_vector_list:
            # print("obstacle vector:", vector)
            # Weigh obstacle vector based on distnace from the robot. The weight is
            # higher if the obstacle is closer. 
            obstacle_weight = (3-abs(euclidean_distance(vector, self.robot_pose)))*0.5
            vector.scale_weight(obstacle_weight,obstacle_weight)
            # Subtracting obstacle vector from heading because the robot should move away from obstacle
            heading = heading.subtract(vector) 
        return heading

    def run_loop(self):
        '''
        Main robot control loop. Adjusts and updates Twist. 
        '''
        vel_msg = Twist()
        if abs(euclidean_distance(self.goal, self.robot_pose)) > 0.5:
            vel_msg.linear.x = 0.15 # Set forward velocity
            # Control angular velocity of the robot based on difference between heading
            # and the current direction the robot is facing
            if (self.heading.x < 0 and abs(self.heading.x + self.yaw) < 1):
                vel_msg.angular.z = 0.15
            elif (self.heading.x > 0 and abs(self.heading.x - self.yaw) < 1):
                vel_msg.angular.z = -0.15
            self.pub_velocity.publish(vel_msg)

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

    def print_obstacle_vectors(self, obstacle_vector_list):
        list = []
        for vector in obstacle_vector_list:
            list.append([vector.x, vector.y])
        print ("obstacle vectors:", list)

#----- helper functions -----------

def euler_from_quaternion(quaternion):
    """
    Converts quaternion to euler roll, pitch, yaw. 
    quaternion = [x, y, z, w]
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

def euclidean_distance(pt1, pt2):
    return math.dist([pt1.x, pt1.y], [pt2.x, pt2.y])

def polar2cart(degree, distance):
    y = distance * math.sin(math.pi * (degree/180))
    x = distance * math.cos(math.pi * (degree/180))
    return x, y


def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = ObstacleAvoiderNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()