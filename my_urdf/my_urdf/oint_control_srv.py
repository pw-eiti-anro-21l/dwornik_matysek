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
from tutorial_interfaces.srv import Interpolation2
import time
import math


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - \
        cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + \
        sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - \
        sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + \
        sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def fun(x, x0, x1, y0, y1):
    iloraz = (x - x0) / (x1 - x0)
    ret = (1 - iloraz) * y0 + iloraz * y1 + iloraz * (1 - iloraz) * \
        ((1 - iloraz) * (-y1 + y0) + iloraz * (y1 - y0))
    return ret


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            Interpolation2, 'interpolacja', self.interpolacja)
        yaml_file = os.path.join(
            get_package_share_directory('my_urdf'), "param.yaml")
        with open(yaml_file, 'r') as stream:
            try:
                self.param = yaml.load(stream, Loader=yaml.FullLoader)
                self.firstlink = self.param.get("firstlink")
                self.secondlink = self.param.get("secondlink")
            except:
                pass

    def interpolacja(self, request, response):
      #  self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)

        if(request.method == 0):
            self.interpolacja_liniowa(request)
        if(request.method == 1):
            self.interpolacja_nieliniowa(request)

        response.response = "sukces!"
        return response

    def interpolacja_liniowa(self, request):
        sample_time = 0.1
        total_time = request.time
        steps = round(total_time / sample_time)
        i = 0
        position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        a = ((request.x) * sample_time) / total_time
        b = ((request.y) * sample_time) / total_time
        c = ((request.z) * sample_time) / total_time
        d = ((request.roll) * sample_time) / total_time
        e = ((request.pitch) * sample_time) / total_time
        f = ((request.yaw) * sample_time) / total_time

        markerArray = MarkerArray()
        qos_profile1 = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/marker_pose', qos_profile1)

        self.publisher_ = self.create_publisher(
            PoseStamped, '/pose_stamped_lab4', 10)
        pose = PoseStamped()

        while (i < steps):
            i += 1
            pose.header.stamp = ROSClock().now().to_msg()
            pose.header.frame_id = "base_link"
            position[0] = position[0] + a
            position[1] = position[1] + b
            position[2] = position[2] + c
            position[3] = position[3] + d
            position[4] = position[4] + e
            position[5] = position[5] + f

            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation = euler_to_quaternion(
                position[3], position[4], position[5])
            self.publisher_.publish(pose)

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
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.id = i
            markerArray.markers.append(marker)
            self.marker_pub.publish(markerArray)

            time.sleep(sample_time)

    def interpolacja_nieliniowa(self, request):
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        sample_time = 0.1
        total_time = request.time
        steps = round(total_time / sample_time)
        i = 0
        position = [self.firstlink+self.secondlink, 0.0, 0.25, 0.0, 0.0, 0.0]

        # każdy ze stawów będzie poruszać się zgodnie z równaniem
        # y = a*x^3 + b*x^2 + c*x + d
        a_1 = -2 * (request.x - position[0]) / (total_time**3)
        a_2 = -2 * (request.y - position[1]) / (total_time**3)
        a_3 = -2 * (request.z - position[2]) / (total_time**3)
        a_r = -2 * (request.roll - position[3]) / (total_time**3)
        a_p = -2 * (request.pitch - position[4]) / (total_time**3)
        a_y = -2 * (request.yaw - position[5]) / (total_time**3)

        b_1 = 3 * (request.x - position[0]) / (total_time**2)
        b_2 = 3 * (request.y - position[1]) / (total_time**2)
        b_3 = 3 * (request.z - position[2]) / (total_time**2)
        b_r = 3 * (request.roll - position[3]) / (total_time**2)
        b_p = 3 * (request.pitch - position[4]) / (total_time**2)
        b_y = 3 * (request.yaw - position[5]) / (total_time**2)

        c_1 = 0
        c_2 = 0
        c_3 = 0
        c_r = 0
        c_p = 0
        c_y = 0

        d_1 = position[0]
        d_2 = position[1]
        d_3 = position[2]
        d_r = position[3]
        d_p = position[4]
        d_y = position[5]

        markerArray = MarkerArray()
        qos_profile1 = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/marker_pose', qos_profile1)

        self.publisher_ = self.create_publisher(
            PoseStamped, '/pose_stamped_lab4', 10)
        pose = PoseStamped()
        marker = Marker()
        marker.action = marker.DELETEALL

        while (i < steps):
            joint_state = JointState()
            i += 1
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['base_to_second',
                                'second_to_third', 'linear_joint']
            pose.header.stamp = ROSClock().now().to_msg()
            pose.header.frame_id = "base_link"
            position[0] = a_1 * (i * sample_time)**3 + b_1 * \
                (i * sample_time)**2 + c_1 * (i * sample_time) + d_1
            position[1] = a_2 * (i * sample_time)**3 + b_2 * \
                (i * sample_time)**2 + c_2 * (i * sample_time) + d_2
            position[2] = a_3 * (i * sample_time)**3 + b_3 * \
                (i * sample_time)**2 + c_3 * (i * sample_time) + d_3
            position[3] = a_r * (i * sample_time)**3 + b_r * \
                (i * sample_time)**2 + c_r * (i * sample_time) + d_r
            position[4] = a_p * (i * sample_time)**3 + b_p * \
                (i * sample_time)**2 + c_p * (i * sample_time) + d_p
            position[5] = a_y * (i * sample_time)**3 + b_y * \
                (i * sample_time)**2 + c_y * (i * sample_time) + d_y

            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation = euler_to_quaternion(
                position[3], position[4], position[5])
            self.publisher_.publish(pose)

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
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.id = i
            markerArray.markers.append(marker)
            self.marker_pub.publish(markerArray)

            time.sleep(sample_time)


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
