import rclpy
from rclpy.node import Node
import curses, time
from geometry_msgs.msg import Twist
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType



def input_char(message):
    try:
        win = curses.initscr()
        win.addstr(0, 0, message)
        i=0
        while i<21:
            win.nodelay(True)
            ch = win.getch()
            if ch in range(32, 127):
                #curses.endwin()
                return chr(ch)
            time.sleep(0.05)
            i=i+1
    except: raise
    finally:
        try:
            curses.endwin()
        except:
             pass
    return 0

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 2)
        self.declare_parameter('dol','s')
        self.declare_parameter('gora','w')
        self.declare_parameter('lewo','a')
        self.declare_parameter('prawo','d')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.control)

    def get_param(self):
        my_param1 = self.get_parameter('dol').get_parameter_value().string_value
        my_param2 = self.get_parameter('gora').get_parameter_value().string_value
        my_param3 = self.get_parameter('lewo').get_parameter_value().string_value
        my_param4 = self.get_parameter('prawo').get_parameter_value().string_value
        return [my_param1,my_param2,my_param3,my_param4]
    
    def control(self):
        msg = Twist()
        params = self.get_param()
        c = input_char("sterowanie\n")
        if (c == params[0]):
            msg.linear.x = -1.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
        elif (c == params[1]):
            msg.linear.x = 1.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
        elif (c == params[2]):
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.publisher_.publish(msg)
        elif (c == params[3]):
            msg.linear.x = 0.0
            msg.angular.z = -1.0
            self.publisher_.publish(msg)
        else:
            return
        self.control()
    
def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

