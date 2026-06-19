"""
자율 순찰 노드 (patrol_node)

순찰 로직:
  1차: 등록된 모든 좌석 (지각 학생 우선)
  2차: 1차 미감지 좌석만 재확인 (patrol_interval_sec 대기)
  3차: 2차 미감지 좌석만 최종 확인 (patrol_interval_sec 대기)

Nav2 실패: nav_retry_delay_sec 대기 후 최대 max_nav_retries 회 재시도, 초과 시 스킵
CSV:  ~/ros2_ws/attendance_logs/attendance_YYYYMMDD_HHMMSS.csv
"""

import csv
import math
import os
import yaml
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

from attendance_msgs.msg import AttendanceMsg, SeatTarget, DetectionResult, PatrolReport


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self.declare_parameter('max_rounds', 3)
        self.declare_parameter('patrol_interval_sec', 300.0)
        self.declare_parameter('detection_timeout_sec', 15.0)
        self.declare_parameter('nav_retry_delay_sec', 3.0)
        self.declare_parameter('max_nav_retries', 3)

        self.max_rounds = self.get_parameter('max_rounds').value
        self.patrol_interval = self.get_parameter('patrol_interval_sec').value
        self.detection_timeout = self.get_parameter('detection_timeout_sec').value
        self.nav_retry_delay = self.get_parameter('nav_retry_delay_sec').value
        self.max_nav_retries = self.get_parameter('max_nav_retries').value

        self.seat_positions = self._load_seat_positions()

        # {seat_id: {'student_id': str, 'status': str, 'registered_at': float}}
        self.registered: dict[str, dict] = {}

        # {(seat_id, round): bool}
        self.results: dict[tuple, bool] = {}

        # {(seat_id, round): float}  — 회차별 신뢰도 (CSV용)
        self.confidences: dict[tuple, float] = {}

        self.current_round = 0
        self.patrol_queue: list[str] = []
        self.current_target: dict | None = None
        self.awaiting_detection = False
        self.detect_start_time = None
        self.round_end_time = None

        self.nav_retry_count = 0
        self._retry_timer = None

        self.csv_path = self._init_csv()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(AttendanceMsg, '/attendance', self._on_attendance, 10)
        self.create_subscription(DetectionResult, '/seat_occupied', self._on_detection, 10)

        self.target_pub = self.create_publisher(SeatTarget, '/patrol_target', 10)
        self.status_pub = self.create_publisher(PatrolReport, '/attendance_status', 10)

        self.create_service(Trigger, '/start_patrol', self._start_patrol_cb)
        self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            '순찰 노드 준비.\n'
            '  순찰 시작: ros2 service call /start_patrol std_srvs/srv/Trigger'
        )

    # ── 초기화 ────────────────────────────────────────────────────────────────

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

    def _init_csv(self) -> str:
        log_dir = os.path.expanduser('~/ros2_ws/attendance_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(log_dir, f'attendance_{ts}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow(self._csv_header())
        self.get_logger().info(f'출석 로그: {path}')
        return path

    def _csv_header(self) -> list:
        header = ['학번', '좌석번호', '입력시각']
        for n in range(1, self.max_rounds + 1):
            header += [f'{n}차순찰', f'{n}차신뢰도']
        header.append('최종상태')
        return header

    # ── 출석 등록 ─────────────────────────────────────────────────────────────

    def _on_attendance(self, msg: AttendanceMsg):
        self.registered[msg.seat_id] = {
            'student_id': msg.student_id,
            'status': msg.status,
            'registered_at': msg.timestamp,
        }
        self.get_logger().info(
            f'등록: 학번={msg.student_id}, 좌석={msg.seat_id}, 상태={msg.status}')

    # ── 순찰 시작 ─────────────────────────────────────────────────────────────

    def _start_patrol_cb(self, request, response):
        if self.current_round > 0:
            response.success = False
            response.message = '이미 순찰 중입니다.'
            return response
        if not self.registered:
            response.success = False
            response.message = '등록된 학생이 없습니다.'
            return response

        self.current_round = 1
        self.patrol_queue = self._build_queue(round_num=1)
        self.get_logger().info(
            f'1차 순찰 시작 — {len(self.patrol_queue)}개 좌석 (지각 학생 우선)')
        self._navigate_next()

        response.success = True
        response.message = f'순찰 시작. 대상: {len(self.registered)}명'
        return response

    def _build_queue(self, round_num: int) -> list[str]:
        """1차: 전체(지각 우선), 2/3차: 이전 회차 미감지 좌석만.

        기본값을 False로 설정 — Nav 실패로 results에 기록이 없는 좌석도
        "미확인"으로 간주해 다음 회차 재확인 대상에 포함시킨다.
        True로 두면 Nav 실패 좌석이 "착석 확인됨"으로 오판되어 FPR이 높아진다.
        """
        if round_num == 1:
            candidates = list(self.registered.keys())
        else:
            prev = round_num - 1
            candidates = [
                s for s in self.registered
                if not self.results.get((s, prev), False)
            ]

        late = [s for s in candidates if self.registered[s]['status'] == 'late']
        others = [s for s in candidates if self.registered[s]['status'] != 'late']
        return late + others

    # ── 순찰 흐름 ─────────────────────────────────────────────────────────────

    def _navigate_next(self):
        if not self.patrol_queue:
            self._finish_round()
            return

        seat_id = self.patrol_queue.pop(0)

        if seat_id not in self.seat_positions:
            self.get_logger().warn(f'좌석 {seat_id} 위치 미정의 — 건너뜀')
            self._navigate_next()
            return

        self.current_target = {'seat_id': seat_id}
        self.awaiting_detection = False
        self.nav_retry_count = 0

        self.get_logger().info(f'[{self.current_round}차] 좌석 {seat_id}로 이동')
        self._send_nav_goal(seat_id)

    def _send_nav_goal(self, seat_id: str):
        pos = self.seat_positions[seat_id]

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 서버 응답 없음')
            self._handle_nav_failure(seat_id)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pos['x'])
        goal.pose.pose.position.y = float(pos['y'])
        yaw = float(pos.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            seat_id = self.current_target['seat_id']
            self.get_logger().warn(f'네비게이션 목표 거부: {seat_id}')
            self._handle_nav_failure(seat_id)
            return
        handle.get_result_async().add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future):
        seat_id = self.current_target['seat_id']

        if hasattr(future.result(), 'status') and future.result().status != 4:
            self.get_logger().warn(f'네비게이션 실패: {seat_id}')
            self._handle_nav_failure(seat_id)
            return

        self.nav_retry_count = 0
        self.get_logger().info(f'좌석 {seat_id} 도착 → 감지 시작')

        msg = SeatTarget()
        msg.seat_id = seat_id
        msg.student_id = self.registered[seat_id]['student_id']
        msg.patrol_round = self.current_round
        pos = self.seat_positions[seat_id]
        msg.pos_x = float(pos['x'])
        msg.pos_y = float(pos['y'])
        msg.pos_yaw = float(pos.get('yaw', 0.0))
        self.target_pub.publish(msg)

        self.awaiting_detection = True
        self.detect_start_time = self.get_clock().now()

    def _handle_nav_failure(self, seat_id: str):
        if self.nav_retry_count < self.max_nav_retries:
            self.nav_retry_count += 1
            self.get_logger().warn(
                f'재시도 {self.nav_retry_count}/{self.max_nav_retries} '
                f'({self.nav_retry_delay:.0f}초 후)')
            self._retry_timer = self.create_timer(
                self.nav_retry_delay, lambda: self._do_retry(seat_id))
        else:
            self.get_logger().error(
                f'좌석 {seat_id} 재시도 초과 — 스킵 (다음 회차에서 재확인 대상 포함)')
            self.nav_retry_count = 0
            self._navigate_next()

    def _do_retry(self, seat_id: str):
        if self._retry_timer:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._send_nav_goal(seat_id)

    # ── 감지 결과 ─────────────────────────────────────────────────────────────

    def _on_detection(self, msg: DetectionResult):
        if not self.awaiting_detection or self.current_target is None:
            return
        if msg.seat_id != self.current_target['seat_id']:
            return
        if msg.patrol_round != self.current_round:
            return

        self.results[(msg.seat_id, self.current_round)] = msg.person_detected
        self.confidences[(msg.seat_id, self.current_round)] = msg.confidence

        flag = '✓ 착석' if msg.person_detected else '✗ 미착석'
        self.get_logger().info(
            f'  [{self.current_round}차] {msg.seat_id}: {flag} '
            f'(신뢰도 {msg.confidence:.0%})')

        self.awaiting_detection = False
        self._navigate_next()

    # ── 회차 완료 ─────────────────────────────────────────────────────────────

    def _finish_round(self):
        self.get_logger().info(f'{self.current_round}차 순찰 완료')

        if self.current_round >= self.max_rounds:
            self._publish_final_reports()
            self.current_round = 0
            return

        next_queue = self._build_queue(round_num=self.current_round + 1)

        if not next_queue:
            self.get_logger().info('재확인 대상 없음 → 전원 출석 확정')
            self._publish_final_reports()
            self.current_round = 0
            return

        self.current_round += 1
        self.patrol_queue = next_queue
        self.round_end_time = self.get_clock().now()
        self.get_logger().info(
            f'{self.patrol_interval / 60:.0f}분 후 {self.current_round}차 순찰 시작 '
            f'(재확인 대상: {len(next_queue)}개)')

    # ── 감시 타이머 ───────────────────────────────────────────────────────────

    def _watchdog(self):
        if self.awaiting_detection and self.detect_start_time:
            elapsed = (self.get_clock().now() - self.detect_start_time).nanoseconds / 1e9
            if elapsed > self.detection_timeout:
                seat_id = self.current_target['seat_id']
                self.get_logger().warn(f'감지 타임아웃: {seat_id} — 미착석으로 처리')
                self.results[(seat_id, self.current_round)] = False
                self.awaiting_detection = False
                self._navigate_next()

        if self.round_end_time and self.patrol_queue:
            elapsed = (self.get_clock().now() - self.round_end_time).nanoseconds / 1e9
            if elapsed >= self.patrol_interval:
                self.round_end_time = None
                self.get_logger().info(f'{self.current_round}차 순찰 시작')
                self._navigate_next()

    # ── 최종 결과 발행 + CSV 저장 ─────────────────────────────────────────────

    def _publish_final_reports(self):
        self.get_logger().info('\n' + '=' * 45)
        self.get_logger().info('최종 출석 판정')
        self.get_logger().info('=' * 45)

        label = {
            'present': '출석', 'late': '지각',
            'suspicious': '대리출석의심', 'absent': '결석',
        }

        def fmt(v):
            return '✓' if v is True else ('✗' if v is False else '-')

        def conf(seat, n):
            v = self.confidences.get((seat, n))
            return f'{v:.0%}' if v is not None else '-'

        csv_rows = []

        # 등록된 학생 판정
        for seat_id, info in self.registered.items():
            student_id = info['student_id']
            reg_time = datetime.fromtimestamp(info['registered_at']).strftime('%H:%M:%S')

            r = [self.results.get((seat_id, n), None) for n in range(1, self.max_rounds + 1)]
            detected = sum(1 for v in r if v is True)

            if detected == 0:
                final_status = 'suspicious'
            elif info['status'] == 'late':
                final_status = 'late'
            else:
                final_status = 'present'

            report = PatrolReport()
            report.student_id = student_id
            report.seat_id = seat_id
            report.total_patrols = self.max_rounds
            report.detected_count = detected
            report.final_status = final_status
            report.stamp = self.get_clock().now().to_msg()
            self.status_pub.publish(report)

            self.get_logger().info(
                f'  {student_id}  좌석={seat_id}  '
                f'감지={detected}/{self.max_rounds}  → {label[final_status]}')

            round_cols = []
            for n in range(1, self.max_rounds + 1):
                v = self.results.get((seat_id, n), None)
                round_cols += [fmt(v), conf(seat_id, n)]

            csv_rows.append([student_id, seat_id, reg_time] + round_cols + [final_status])

        # 미등록 좌석 → 결석 처리
        for seat_id in self.seat_positions:
            if seat_id in self.registered:
                continue
            self.get_logger().info(f'  미등록  좌석={seat_id}  → 결석')
            report = PatrolReport()
            report.student_id = '미등록'
            report.seat_id = seat_id
            report.total_patrols = 0
            report.detected_count = 0
            report.final_status = 'absent'
            report.stamp = self.get_clock().now().to_msg()
            self.status_pub.publish(report)

            absent_cols = ['-', '-'] * self.max_rounds
            csv_rows.append(['미등록', seat_id, '-'] + absent_cols + ['absent'])

        with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerows(csv_rows)

        self.get_logger().info('=' * 45)
        self.get_logger().info(f'CSV 저장: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
