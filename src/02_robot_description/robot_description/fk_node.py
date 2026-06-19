import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class FKChecker(Node):

    def __init__(self):
        super().__init__('fk_checker')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Print end-effector position every 0.5s
        self.timer = self.create_timer(0.5, self.check_fk)

    def check_fk(self):
        try:
            # Get transform from base_link to link_2 (end-effector)
            t = self.tf_buffer.lookup_transform(
                'base_link',   # target frame
                'link_2',      # source frame
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            self.get_logger().info(
                f'End-effector position — x: {x:.4f}  y: {y:.4f}  z: {z:.4f}'
            )

        except Exception as e:
            self.get_logger().warn(f'TF2 lookup failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FKChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()