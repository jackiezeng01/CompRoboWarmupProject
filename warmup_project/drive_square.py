import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1
        self.count = 0
        self.turn = 0
        self.timer = self.create_timer(timer_period, self.run_loop)

    def run_loop(self):
        if self.count == 0:
            twt = Twist()
            twt.linear.x = 0.5
            self.pub.publish(twt)
        elif self.count == 20:
            twt = Twist()
            twt.linear.x = 0.0
            self.pub.publish(twt)
        elif self.count == 30:
            twt = Twist()
            twt.angular.z = 0.786
            self.pub.publish(twt)
        elif self.count == 50:
            twt = Twist()
            twt.angular.z = 0.0
            self.pub.publish(twt)
        
        self.count += 1

        if self.count == 60:
            if self.turn == 3:
                self.destroy_node()
            else:
                self.count = 0
                self.turn += 1

def main(args=None):
    rclpy.init(args=args)         # Initialize communication with ROS
    node = DriveSquareNode()   # Create our Node
    rclpy.spin(node)              # Run the Node until ready to shutdown
    rclpy.shutdown()              # cleanup

if __name__ == '__main__':
    main()