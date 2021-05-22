import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
from PyKDL import *
from math import pi, acos, sin, cos, asin


class Ikin(Node):
    def __init__(self):
        super().__init__('ikin')

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
        self.x = 0
        self.y = 0
        self.z = 0
        self.joint_state = JointState()
        self.joint_state.name = ['base_to_second',
                            'second_to_third', 'linear_joint']
        yaml_file = os.path.join(
            get_package_share_directory('my_urdf'), "param.yaml")
        with open(yaml_file, 'r') as stream:
            try:
                self.param = yaml.load(stream, Loader=yaml.FullLoader)
                self.firstlink = self.param.get("firstlink")
                self.secondlink = self.param.get("secondlink")
            except:
                pass


    def listener_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        c_2 = (self.x**2 + self.y**2 - self.firstlink**2 -
               self.secondlink**2) / (2 * self.firstlink * self.secondlink)

        if(c_2>1 or c_2<-1):
            print("Cringe.")
        else:
            sigma_2 = acos(c_2)
            sigma_1 = 0
            k_1 = self.firstlink + self.secondlink*c_2
            k_2 = self.secondlink*sin(sigma_2)

            s_1 = -(k_2*self.x+self.y*k_1)/(k_1**2+k_2**2)
            if(s_1>1 or s_1<-1):
                print("Cringe.")
            else:
                sigma_1 = asin(s_1)
                self.joint_state.position = [sigma_1, sigma_2, 0.4-self.z]
                print(self.joint_state.position)
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.publisher.publish(self.joint_state)



def main(args=None):
    rclpy.init(args=args)

    ikin = Ikin()
    rclpy.spin(ikin)

    ikin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
