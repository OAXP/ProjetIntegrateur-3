from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class StaticMapBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'merge_map'
       t.child_frame_id = 'limo_0/limo/odom'
       t.transform.translation.x = 0.0
       t.transform.translation.y = 0.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       t1 = TransformStamped()

       t1.header.stamp = self.get_clock().now().to_msg()
       t1.header.frame_id = 'merge_map'
       t1.child_frame_id = 'limo_1/limo/odom'
       t1.transform.translation.x = 0.0
       t1.transform.translation.y = -1.0
       t1.transform.translation.z = 0.0
       t1.transform.rotation.x = 0.0
       t1.transform.rotation.y = 0.0
       t1.transform.rotation.z = 0.0
       t1.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)
       self.tf_broadcaster.sendTransform(t1)


def main():
    rclpy.init()
    node = StaticMapBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()