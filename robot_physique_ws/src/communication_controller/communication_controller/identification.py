import rclpy
from communication_interfaces.msg import RobotIdentification
from playsound import playsound
from socket import gethostname


def handle_identification(msg):
    current_hostname = gethostname()

    if msg.robot_id == 'all' or msg.robot_id == current_hostname:
        playsound("../sounds/identify.mp3", False)

        print(f"Playing sound for robot_id {msg.robot_id}!")
    else:
        print(f"{msg.robot_id} does not match {current_hostname}")


def main():
    rclpy.init()
    node = rclpy.create_node('identification_node')
    subscriber = node.create_subscription(RobotIdentification, '/identification', handle_identification, 10)
    print(f"Ready to play sound. My hostname is {gethostname()}")
    rclpy.spin(node)


if __name__ == '__main__':
    main()
