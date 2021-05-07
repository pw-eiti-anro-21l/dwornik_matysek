import sys
from tutorial_interfaces.srv import Interpolation
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Interpolation, 'interpolacja')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Interpolation.Request()

    def send_request(self):
        try:
            if(float(sys.argv[1])>3.14 or float(sys.argv[1]) < -3.14):
                self.get_logger().info('Niepoprawna wartość położenia stawu 1.') 
                raise ValueError()
            else:
                self.req.joint1 = float(sys.argv[1])

            if(float(sys.argv[2])>0.85 or float(sys.argv[2]) < -0.85):
                self.get_logger().info('Niepoprawna wartość położenia stawu 2.')                
                raise ValueError()
            else:
                self.req.joint2= float(sys.argv[2])

            if(float(sys.argv[3])>0.1 or float(sys.argv[3]) < 0):
                self.get_logger().info('Niepoprawna wartość położenia stawu 3.')
                raise ValueError()
            else:
                self.req.joint3 = float(sys.argv[3])

            if(float(sys.argv[4])<=0):
                self.get_logger().info('Niepoprawna wartość czasu')
                raise ValueError()
            else:
                self.req.time = float(sys.argv[4])
        except ValueError:
            raise Exception()

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(response.response)
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()