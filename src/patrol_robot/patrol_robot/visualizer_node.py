"""
시각화 노드 (visualizer_node)

/attendance_status 구독 → RViz2 MarkerArray로 좌석별 출석 상태 표시
  초록: 출석 (present)
  노랑: 지각 (late)
  빨강: 대리출석 의심 (suspicious)
  회색: 결석 (absent)
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from attendance_msgs.msg import PatrolReport


STATUS_COLOR = {
    'present':    ColorRGBA(r=0.0, g=0.9, b=0.0, a=0.9),   # 초록
    'late':       ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),   # 노랑
    'suspicious': ColorRGBA(r=0.9, g=0.0, b=0.0, a=0.9),   # 빨강
    'absent':     ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.9),   # 회색
}
STATUS_LABEL = {
    'present': '출석', 'late': '지각',
    'suspicious': '대리출석의심', 'absent': '결석',
}

# 마커 lifetime: 노드가 멈추면 2초 후 RViz2에서 자동 사라짐
_LIFETIME = DurationMsg(sec=2, nanosec=0)


class VisualizerNode(Node):

    def __init__(self):
        super().__init__('visualizer_node')

        self.seat_positions = self._load_seat_positions()
        # {seat_id: PatrolReport}
        self.seat_status: dict[str, PatrolReport] = {}

        self.create_subscription(
            PatrolReport, '/attendance_status', self._on_report, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/attendance_markers', 10)

        # 1초마다 마커 갱신 발행 (RViz2가 새로 연결될 때도 표시되도록)
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            '시각화 노드 준비. RViz2에서 /attendance_markers (MarkerArray) 추가하세요.')

    def _load_seat_positions(self) -> dict:
        try:
            path = os.path.join(
                get_package_share_directory('patrol_robot'),
                'config', 'seat_positions.yaml')
            with open(path) as f:
                return yaml.safe_load(f).get('seats', {})
        except Exception as e:
            self.get_logger().error(f'좌석 위치 로드 실패: {e}')
            return {}

    def _on_report(self, msg: PatrolReport):
        self.seat_status[msg.seat_id] = msg
        label = STATUS_LABEL.get(msg.final_status, msg.final_status)
        self.get_logger().info(f'마커 갱신: {msg.seat_id} → {label}')
        self._publish_markers()

    def _publish_markers(self):
        if not self.seat_status:
            return

        array = MarkerArray()
        marker_id = 0

        for seat_id, report in self.seat_status.items():
            if seat_id not in self.seat_positions:
                continue

            pos = self.seat_positions[seat_id]
            color = STATUS_COLOR.get(report.final_status, STATUS_COLOR['absent'])
            now = self.get_clock().now().to_msg()

            # 구체 마커 (좌석 위치에 표시)
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'seat_status'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(
                x=float(pos.get('seat_x', pos['x'])), y=float(pos.get('seat_y', pos['y'])), z=0.5)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
            sphere.color = color
            sphere.lifetime = _LIFETIME
            array.markers.append(sphere)
            marker_id += 1

            # 텍스트 마커 (학번 + 상태)
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'seat_labels'
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(
                x=float(pos.get('seat_x', pos['x'])), y=float(pos.get('seat_y', pos['y'])), z=1.0)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label = STATUS_LABEL.get(report.final_status, report.final_status)
            text.text = f'{report.student_id}\n[{seat_id}] {label}'
            text.lifetime = _LIFETIME
            array.markers.append(text)
            marker_id += 1

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
