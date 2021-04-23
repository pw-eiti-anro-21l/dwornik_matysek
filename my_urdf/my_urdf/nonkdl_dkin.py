from math import sin, cos, pi
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
import json
from rclpy.clock import ROSClock

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class nonkdl_dkin(Node):

  def __init__(self):
    super().__init__('nonkdl_dkin')
    
    self.subscription = self.create_subscription(
      JointState,
      'joint_states',
      self.listener_callback,
      10)
    self.subscription

  def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.position[0])
    self.publisher_ = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', 10)
    msg = PoseStamped()
    msg.header.stamp = ROSClock().now().to_msg()
    msg.header.frame_id = "base_link"
    msg.pose.position.x=2.0
    msg.pose.position.y=2.0
    msg.pose.position.z=2.0
    msg.pose.orientation = euler_to_quaternion(0,pi,0)
    self.publisher_.publish(msg)
    
  
  def timer_callback(self):
    msg = PoseStamped()
    msg.pose.position.x=1.0
    msg.pose.position.y=1.0
    msg.pose.position.z=1.0
    self.publisher_.publish(msg)
  
def main(args=None):
  rclpy.init(args=args)
  node = nonkdl_dkin()
  rclpy.spin(node)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
