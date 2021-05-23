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
        self.elipsa=0


    def timer_callback(self):
        msg = PoseStamped()
        l=80
        if self.elipsa:
            a=1
            b=2
            if self.i<l/2:
                x=b-self.i*b*4/l
                msg.pose.position.x = x
                msg.pose.position.y = (1-x*x/(b*b))*a*a
            else:
                x=-b+(self.i-l/2)*4/l
                msg.pose.position.x = x
                msg.pose.position.y = -(1-x*x/(b*b))*a*a
        else:
            a=1.5
            if self.i<(l/8):
                msg.pose.position.x = float(a)
                msg.pose.position.y = float(self.i*a*8/l)
            elif self.i<(3*l/8):
                msg.pose.position.x = float(a-(self.i-l/8)*a*8/l)
                msg.pose.position.y = float(a)
            elif self.i<(5*l/8):
                msg.pose.position.x = float(-a)
                msg.pose.position.y = float(a-(self.i-3*l/8)*a*8/l)
            elif self.i<(7*l/8):
                msg.pose.position.x = float(-a+(self.i-5*l/8)*a*8/l)
                msg.pose.position.y = float(-a)
            else:
                msg.pose.position.x = float(a)
                msg.pose.position.y = float(-a+(self.i-7*l/8)*a*8/l)
        msg.pose.position.z = float(0.1)
        if self.i==(l-1):
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
