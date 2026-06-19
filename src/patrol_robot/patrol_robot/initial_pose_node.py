"""
AMCL 초기 위치 자동 발행 노드

patrol_system.launch.py 실행 시 delay_sec 후 /initialpose 를 1회 발행하여
RViz2에서 "2D Pose Estimate"를 수동으로 클릭하지 않아도 AMCL이 초기화되도록 한다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseNode(Node):

    def __init__(self):
        super().__init__('initial_pose_node')
        self.declare_parameter('x', -4.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay_sec', 5.0)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)

        delay = self.get_parameter('delay_sec').value
        self._timer = self.create_timer(delay, self._publish_once)
        self.get_logger().info(
            f'초기 위치 노드 준비 — {delay:.0f}초 후 /initialpose 발행 예정')

    def _publish_once(self):
        self._timer.cancel()

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # 표준 AMCL 초기 공분산
        cov = [0.0] * 36
        cov[0] = 0.25    # x
        cov[7] = 0.25    # y
        cov[35] = 0.0685 # θ
        msg.pose.covariance = cov

        self._pub.publish(msg)
        self.get_logger().info(f'초기 위치 발행 완료: x={x}, y={y}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
