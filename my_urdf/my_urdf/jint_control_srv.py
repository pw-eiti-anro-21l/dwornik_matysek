import rclpy
from rclpy.node import Node
import os
from math import sin, cos, pi
import yaml
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
        yaml_file = os.path.join(get_package_share_directory('my_urdf'), "param.yaml")
        with open(yaml_file,'r') as stream:
            try:
                self.param = yaml.load(stream,Loader=yaml.FullLoader)
                self.firstlink = self.param.get("firstlink")
                self.secondlink = self.param.get("secondlink")
            except:
                pass

    def interpolacja(self, request, response):
      #  self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        sample_time = 0.1
        total_time = request.time
        steps = round(total_time/sample_time)
        i=0
        position=[0.0,0.0,0.0]
        a = ((request.joint1)*sample_time)/total_time
        b = ((request.joint2)*sample_time)/total_time
        c = ((request.joint3)*sample_time)/total_time

        markerArray = MarkerArray()
        qos_profile1 = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(MarkerArray, '/marker_pose', qos_profile1)

        while (i<steps):
            joint_state = JointState()
            i+=1
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base_to_second', 'second_to_third', 'linear_joint']
            position[0] = position[0]+ a
            position[1] = position[1]+ b
            position[2] = position[2]+ c
            joint_state.position = position
            self.joint_pub.publish(joint_state)
            time.sleep(sample_time)

            marker = Marker()
            marker.header.frame_id = "/base_link"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.firstlink*cos(position[0])+self.secondlink*cos(position[1]+position[0])
            marker.pose.position.y =  self.firstlink*sin(position[0])+self.secondlink*sin(position[1]+position[0])
            marker.pose.position.z =  0.25-position[2]
            marker.id=i
            markerArray.markers.append(marker)
            self.marker_pub.publish(markerArray)

        response.response = "sukces!"
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
