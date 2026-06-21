import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import time
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry


class PlanningPublisher(Node):

    def __init__(self):
        super().__init__('simple_planner_node')
        self.pos = None
        self.publisher_ = self.create_publisher(PointStamped, '/goal_point', 10)
        
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    
    def timer_callback(self):
        R = 5
        if self.pos is not None: 
            msg = PointStamped()
            #msg.header = time.time()
            sampled_point = self.pos + R*np.random.rand() 
            msg.point.x = sampled_point[0]
            msg.point.y = sampled_point[1]
            self.publisher_.publish(msg)
            #self.get_logger().info('Publishing: "%s"' % msg.data)
            


def main(args=None):
    rclpy.init(args=args)

    node = PlanningPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()