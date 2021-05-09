import sys
from tutorial_interfaces.srv import Interpolation2
import rclpy
from rclpy.node import Node

class MinimalClientAsync2(Node):

    def __init__(self):
        super().__init__('minimal_client_async2')
        self.cli = self.create_client(Interpolation2, 'interpolacja')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Interpolation2.Request()

    def send_request(self):

        self.req.x = float(sys.argv[1])
        self.req.y= float(sys.argv[2])
        self.req.z = float(sys.argv[3])
        self.req.roll = float(sys.argv[4])
        self.req.pitch= float(sys.argv[5])
        self.req.yaw = float(sys.argv[6])
        self.req.time = float(sys.argv[7])
        self.req.method = int(sys.argv[8])

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync2()
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
