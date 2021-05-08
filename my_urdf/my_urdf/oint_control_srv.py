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
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
def fun(x, x0, x1, y0, y1):
    iloraz = (x-x0)/(x1-x0)
    ret = (1-iloraz)*y0 + iloraz *y1 +iloraz*(1-iloraz)*((1-iloraz)*(-y1+y0) + iloraz*(y1-y0))  
    return ret

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Interpolation2, 'interpolacja', self.interpolacja)

    def interpolacja(self, request, response):
      #  self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        sample_time = 0.1
        total_time = request.time
        steps = round(total_time/sample_time)
        i=0
        position=[0.0,0.0,0.0,0.0,0.0,0.0]
        a = ((request.x)*sample_time)/total_time
        b = ((request.y)*sample_time)/total_time
        c = ((request.z)*sample_time)/total_time
        d = ((request.roll)*sample_time)/total_time
        e = ((request.pitch)*sample_time)/total_time
        f = ((request.yaw)*sample_time)/total_time

        markerArray = MarkerArray()
        qos_profile1 = QoSProfile(depth=10)

        self.marker_pub = self.create_publisher(MarkerArray, '/marker_pose', qos_profile1)
        

        self.publisher_ = self.create_publisher(PoseStamped, '/pose_stamped_lab4', 10)
        pose = PoseStamped()

        while (i<steps):
            i+=1
            pose.header.stamp = ROSClock().now().to_msg()
            pose.header.frame_id = "base_link"
            # position[0] = position[0]+ a
            # position[1] = position[1]+ b
            # position[2] = position[2]+ c
            # position[3] = position[3]+ d
            # position[4] = position[4]+ e
            # position[5] = position[5]+ f
            position[0] = fun(i,0,steps,0,request.x)
            position[1] = fun(i,0,steps,0,request.y)
            position[2] = fun(i,0,steps,0,request.z)
            position[3] = position[3]+ d
            position[4] = position[4]+ e
            position[5] = position[5]+ f
            
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = position[2]
            pose.pose.orientation = euler_to_quaternion(position[3], position[4] ,position[5])
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
            marker.id=i
            markerArray.markers.append(marker)
            self.marker_pub.publish(markerArray)

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
