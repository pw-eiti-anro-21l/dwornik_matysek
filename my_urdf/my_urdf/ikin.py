import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.clock import ROSClock
from copy import deepcopy
from PyKDL import *
from math import acos, sin, cos, asin, atan2


class Ikin(Node):
    def __init__(self):
        super().__init__('ikin')
        self.marker = Marker()
        self.markerArray = MarkerArray()
        self.marker_pub = self.create_publisher(
            MarkerArray, '/marker_pose', 10)

        self.subscription = self.create_subscription(
            PoseStamped,
            'pose_stamped_lab4',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            JointState,
            "joint_states",
            10
        )
        self.path_publisher = self.create_publisher(
            Path,
            "path_of_robot",
            10
        )
        self.x = 0
        self.y = 0
        self.z = 0
        self.i = 0
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_second',
                                 'second_to_third', 'linear_joint']
        self.path = Path()

        yaml_file = os.path.join(
            get_package_share_directory('my_urdf'), "param.yaml")
        with open(yaml_file, 'r') as stream:
            try:
                self.param = yaml.load(stream, Loader=yaml.FullLoader)
                self.firstlink = self.param.get("firstlink")
                self.secondlink = self.param.get("secondlink")
            except:
                pass
        self.marker_init()

    def marker_init(self):
        self.marker.header.frame_id = "/base_link"
        self.marker.action = self.marker.DELETEALL
        self.marker.header.frame_id = "/base_link"
        self.markerArray.markers.append(self.marker)
        self.marker_pub.publish(self.markerArray)
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0

    def listener_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        c_2 = (self.x**2 + self.y**2 - self.firstlink**2 -
               self.secondlink**2) / (2 * self.firstlink * self.secondlink)

        if(c_2 > 1 or c_2 < -1):
            print("Cringe.")
        else:
            sigma_2 = acos(c_2)
            sigma_1 = 0
            k_1 = self.firstlink + self.secondlink * c_2
            k_2 = self.secondlink * sin(sigma_2)


            sigma_1 = atan2(self.y, self.x) - atan2(k_2, k_1)
            self.joint_state.position = [sigma_1, sigma_2, 0.25 - self.z]
            now = self.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            self.publisher.publish(self.joint_state)

            self.marker.pose.position.x = self.x
            self.marker.pose.position.y = self.y
            self.marker.pose.position.z = self.z - 0.10
            self.marker.id = self.i
            self.marker.type = self.marker.SPHERE
            self.marker.action = self.marker.ADD
            self.markerArray.markers.append(self.marker)
            self.marker_pub.publish(self.markerArray)

            self.i+=1

            # self.path.header.frame_id = "path"
            # self.path.header.stamp = now.to_msg()
            # self.path.poses.append(deepcopy(msg))
            # self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)

    ikin = Ikin()
    rclpy.spin(ikin)

    ikin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
