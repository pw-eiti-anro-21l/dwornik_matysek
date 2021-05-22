#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        swivel2 = 0.
        tinc = degree
        swivel = 0.
        height = 0.
        hinc = 0.005
        tinc2 = tinc / 2

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_second',
                                    'second_to_third', 'linear_joint']
                joint_state.position = [swivel, swivel2, height]

                # update transform
                odom_trans.header.stamp = now.to_msg()

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                swivel2 += tinc
                swivel += tinc2
                if swivel2 < -2 or swivel2 > 2:
                    tinc *= -1
                if swivel < -1.9 or swivel > 1.9:
                    tinc2 *= -1
                height += hinc
                if height > 0.1 or height < 0.0:
                    hinc *= -1

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main():
    node = StatePublisher()


if __name__ == '__main__':
    main()
