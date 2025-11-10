import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from btcpp_ros2_interfaces.srv import StartUpSrv


class ReadySignal(Node):
    def __init__(self):
        super().__init__('main_ready')

        self.ready_sub = self.create_subscription(
            String,
            '/robot/startup/plan',
            self.readyCallback,
            2
        )

        self.ready_srv_client = self.create_client(
            StartUpSrv,
            '/robot/startup/ready_signal'
        )

        self.is_main_ready = False

    def readyCallback(self, msg):
        if msg is not None and not self.is_main_ready:
            self.is_main_ready = True

    def sendReadySignal(self, group_, state_):
        self.get_logger().info('send ready signal')

        # 4:localization
        request = StartUpSrv.Request()
        request.group = group_
        request.state = state_

        self.ready_srv_client.call_async(request)

        self.get_logger().info(f"response: success={state_}, group={group_}")

def main(args=None):
    rclpy.init(args=args)
    node = ReadySignal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
