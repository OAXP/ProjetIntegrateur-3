import rclpy
from rclpy.node import Node
from limo_msgs.msg import LimoStatus

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        self.publisher_ = self.create_publisher(LimoStatus, '/limo_status', 10)
        timer_period = 1/1 # Hz
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LimoStatus()
        msg.battery_voltage = 12.0
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = StatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()