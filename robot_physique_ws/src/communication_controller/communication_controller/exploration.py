import os
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

from sensor_msgs.msg import LaserScan

distance_from_wall = 0.4

class ExplorationSubscriber(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.is_exploring = False
        
        self.subscription = self.create_subscription(Bool, '/exploration', self.exploration_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_profile)

        self.declare_parameter('is_exploring', False)
        self.declare_parameter('start_time', 0.0)


    def exploration_callback(self, msg):
        self.set_parameters([rclpy.Parameter('is_exploring', rclpy.Parameter.Type.BOOL, msg.data)])
        if msg.data and self.is_exploring == False:
            self.start_exploration()
        elif not msg.data and self.is_exploring == True: 
            self.stop_exploration()

    def start_exploration(self):
        self.is_exploring = True
        start_time = time.time()
        self.set_parameters([rclpy.Parameter('start_time', rclpy.Parameter.Type.DOUBLE, start_time)])

    def stop_exploration(self):
        self.is_exploring = False
        start_time = 0.0
        self.set_parameters([rclpy.Parameter('start_time', rclpy.Parameter.Type.DOUBLE, start_time)])
        msg = Twist()
        self.publisher.publish(msg)

    def scan_callback(self, msg):
        if not self.is_exploring:
            return
        # Check if the front of the robot is close to an obstacle
        middle_index = len(msg.ranges) // 2
        # front_distance = min(msg.ranges[middle_index-1 : middle_index+1])
        right_distance = min(msg.ranges[middle_index-15 : middle_index-8])
        left_distance = min(msg.ranges[middle_index+8 : middle_index+15])
        if (right_distance <= distance_from_wall):
            self.rotate_robot('left')
        elif (left_distance < distance_from_wall) and (left_distance <= right_distance):
            self.rotate_robot('right')
        else:
            self.go_forward()

    def rotate_robot(self, direction):
        msg = Twist()
        if direction == 'left':
            msg.angular.z = 0.8
        else:
            msg.angular.z = -0.8
        self.publisher.publish(msg)

    def go_forward(self):
        msg = Twist()
        msg.linear.x = 0.3
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ExplorationSubscriber()
    print("Exploration node created")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
