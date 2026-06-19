import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriber')

        # Subscribe to topic 'topic' with message type String
        # callback is called every time a new message arrives
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.receive_message,
            10)
        self.subscription  # prevent unused variable warning

    def receive_message(self, msg):
        # This callback is invoked by spin() whenever a message is received
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = SubscriberNode()

    # spin() blocks here and processes callbacks until the node is shut down
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
