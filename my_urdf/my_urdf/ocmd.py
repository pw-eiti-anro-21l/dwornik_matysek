import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class Ocmd(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'pose_stamped_lab4', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.elipsa=1


    def timer_callback(self):
        msg = PoseStamped()
        if self.elipsa:
            a=1
            b=2
            if self.i<40:
                x=b-self.i*b/20
                msg.pose.position.x = x
                msg.pose.position.y = (1-x*x/(b*b))*a*a
            elif self.i<80:
                x=-b+(self.i-40)*b/20
                msg.pose.position.x = x
                msg.pose.position.y = -(1-x*x/(b*b))*a*a
        else:
            if self.i<10:
                msg.pose.position.x = float(2)
                msg.pose.position.y = float(self.i*1/5)
            elif self.i<30:
                msg.pose.position.x = float(2-(self.i-10)*2/10)
                msg.pose.position.y = float(2)
            elif self.i<50:
                msg.pose.position.x = float(-2)
                msg.pose.position.y = float(2-(self.i-30)*2/10)
            elif self.i<70:
                msg.pose.position.x = float(-2+(self.i-50)*2/10)
                msg.pose.position.y = float(-2)
            elif self.i<80:
                msg.pose.position.x = float(2)
                msg.pose.position.y = float(-2+(self.i-70)*1/5)
        msg.pose.position.z = float(0.1)
        if self.i==79:
            self.i=-1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.pose.position)
        self.i += 1
        


def main(args=None):
    rclpy.init(args=args)

    ocmd = Ocmd()

    rclpy.spin(ocmd)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ocmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
