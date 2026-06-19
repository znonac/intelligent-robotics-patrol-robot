import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher')

        # Create a publisher on topic 'topic' with message type String
        # Queue size 10: buffer up to 10 messages if subscriber is slow
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Fire publish every 0.5 seconds (2 Hz)
        # Stored as instance variable — otherwise garbage collected
        self.timer = self.create_timer(0.5, self.publish_message)

        # Counter to track how many messages have been published
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)

        # get_logger() is preferred over print() — includes timestamp and node name
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()

    # spin() blocks here and processes callbacks until the node is shut down
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
