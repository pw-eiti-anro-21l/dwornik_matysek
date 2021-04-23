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
from PyKDL import *



class Kdl_dkin(Node):
    def __init__(self):
        super().__init__('Kdl_dkin')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        #Kinematic chain

        chain = Chain()
        base_link__link_1 = Joint(Joint.RotZ)
        frame1 = Frame(Rotation.RPY(0,0,0), Vector(0,0,0.3))
        segment1 = Segment(base_link__link_1,frame1)
        chain.addSegment(segment1)


        link_1__link_2 = Joint(Joint.RotZ)
        frame2 = Frame(Rotation.RPY(0, 0, 0), Vector(1, 0, 0))
        segment2=Segment(link_1__link_2,frame2)
        chain.addSegment(segment2)


        link_2__link_3 = Joint(Joint.TransZ)
        frame3 = Frame(Rotation.RPY(0,3.14,0), Vector(1,0,-0.05))
        segment3=Segment(link_2__link_3,frame3)
        chain.addSegment(segment3)


        #Forward kinematics

        joint_positions=JntArray(3)
        joint_positions[0]= msg.position[0]
        joint_positions[1]= -msg.position[1]
        joint_positions[2]= -msg.position[2]

        # Rekursywny solver kinematyki prostej

        fk=ChainFkSolverPos_recursive(chain)
        finalFrame=Frame()
        fk.JntToCart(joint_positions,finalFrame)

        # Rotational Matrix of the final Frame:
        qua = finalFrame.M.GetQuaternion()

        # End-effector position + effector offset in respect to last joint (joint3):
        tool_offset = Vector(0.1, 0, 0)
        xyz = finalFrame.p + tool_offset



        qos_profile = QoSProfile(depth=10)
        pose_publisher = self.create_publisher(PoseStamped, '/pose_stamped_kdl', qos_profile)


        poses = PoseStamped()
        now = self.get_clock().now()
        poses.header.stamp = ROSClock().now().to_msg()
        poses.header.frame_id = "base_link"

        poses.pose.position.x = xyz[0]
        poses.pose.position.y = xyz[1]
        poses.pose.position.z = xyz[2]
        poses.pose.orientation = Quaternion(w=float(qua[0]), x=float(qua[1]), y=float(qua[2]), z=float(qua[3]))
        pose_publisher.publish(poses)


def main(args=None):
    rclpy.init(args=args)

    kdl = Kdl_dkin()
    rclpy.spin(kdl)

    kdl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
