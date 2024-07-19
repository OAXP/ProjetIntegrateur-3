import rclpy
from rclpy.node import Node
from communication_interfaces.msg import RobotInfo
from rcl_interfaces.srv import GetParameters
from limo_msgs.msg import LimoStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class RobotInfoPublisher(Node):
    def __init__(self):
        super().__init__('robot_info_node')
        self.declare_parameter('robot_id', '')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.total_clients_ = 1 # total clients to get info from
        self.asked_clients_ = 0 # current requested clients
        self.battery_voltage_ = 0.0
        self.position_x_ = 0.0
        self.position_y_ = 0.0
        self.distance_ = 0.0

        self.publisher_ = self.create_publisher(RobotInfo, '/robot_info', 10)
        self.limo_status_sub_ = self.create_subscription(LimoStatus, '/limo_status', self.status_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, self.robot_id + '/limo/odom', self.odom_callback, 10)
        self.scan_sub_ = self.create_subscription(LaserScan, self.robot_id + '/limo/scan', self.scan_callback, 10)
        self.exploration_client = self.create_client(GetParameters, f'/exploration_{self.robot_id}/get_parameters')
        timer_period = 1/1 # Hz
        self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        self.asked_clients_ = 0
        self.msg_ = RobotInfo()
        self.msg_.robot_id = self.robot_id
        self.msg_.battery_voltage = self.battery_voltage_
        self.msg_.distance = self.distance_
        self.msg_.position_x = self.position_x_
        self.msg_.position_y = self.position_y_

        self.get_parameter_async(self.exploration_client, ['is_exploring', 'start_time'], self.is_exploring_callback)
    
    def publish_final_message(self):
        if self.asked_clients_ == self.total_clients_:
            self.publisher_.publish(self.msg_)
            self.get_logger().info(f"Published robot info : {str(self.msg_)}")
        else:
            pass

    def get_parameter_async(self, client, parameter_name, callback):
        request = GetParameters.Request()
        request.names = parameter_name

        client.wait_for_service()

        future = client.call_async(request)
        future.add_done_callback(callback)

    def is_exploring_callback(self, future):
        self.asked_clients_ = self.asked_clients_ + 1
        is_exploring = future.result().values[0].bool_value
        start_time = future.result().values[1].double_value
        self.msg_.start_time = start_time
        if is_exploring:
            self.msg_.status = "En mission"
        else:
            self.msg_.status = "En attente de mission"

        self.publish_final_message()


    def status_callback(self, msg: LimoStatus):
        self.battery_voltage_ = msg.battery_voltage

    def odom_callback(self, msg: Odometry):
        self.position_x_ = msg.pose.pose.position.x
        self.position_y_ = msg.pose.pose.position.y

    def scan_callback(self, msg: LaserScan):
        self.distance_ = msg.ranges[len(msg.ranges)//2]

def main():
    rclpy.init()
    # TODO subscribe to all info topics
    node = RobotInfoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
