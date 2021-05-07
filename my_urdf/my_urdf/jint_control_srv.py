import rclpy
from rclpy.node import Node
import os
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rclpy.clock import ROSClock
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tutorial_interfaces.srv import Interpolation
import time
import math 

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Interpolation, 'interpolacja', self.interpolacja)

    def interpolacja(self, request, response):
      #  self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        sample_time = 0.1
        total_time = request.time
        steps = round(total_time/sample_time)
        i=0
        position=[0.0,0.0,0.0]
        while (i<steps):
            joint_state = JointState()
            i+=1
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base_to_second', 'second_to_third', 'linear_joint']
            position[0] = position[0]+ ((request.joint1)*sample_time)/total_time
            position[1] = position[1]+ ((request.joint2)*sample_time)/total_time
            position[2] = position[2]+ ((request.joint3)*sample_time)/total_time
            joint_state.position = position
            self.joint_pub.publish(joint_state)
            time.sleep(sample_time)

        response.response = "sukces!"
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
