import threading
import time

import rclpy
from rclpy.node import Node

from attendance_msgs.msg import AttendanceMsg

VALID_SEATS = {'A1', 'A2', 'B1', 'B2'}


class AttendanceNode(Node):
    def __init__(self):
        super().__init__('attendance_node')
        self.publisher_ = self.create_publisher(AttendanceMsg, '/attendance', 10)
        self.start_time = time.time()
        self.get_logger().info('출석 등록 시스템 시작 (Ctrl+C로 종료)')
        # Run blocking input() in a background thread so rclpy.spin() keeps running
        self._input_thread = threading.Thread(target=self._registration_loop, daemon=True)
        self._input_thread.start()

    def _registration_loop(self):
        while rclpy.ok():
            print('\n=== 출석 등록 ===')
            try:
                student_id = input('학번: ').strip()
                if not student_id:
                    continue
                seat_id = input('좌석번호 (예: A1): ').strip().upper()
                if not seat_id:
                    continue
                if seat_id not in VALID_SEATS:
                    print(f'유효하지 않은 좌석번호입니다. 사용 가능: {", ".join(sorted(VALID_SEATS))}')
                    continue
            except (EOFError, KeyboardInterrupt):
                break

            elapsed_min = (time.time() - self.start_time) / 60
            if elapsed_min <= 10:
                status = 'present'
            elif elapsed_min <= 30:
                status = 'late'
            else:
                print('출석 등록 가능 시간이 지났습니다 (강의 시작 30분 이후).')
                continue

            msg = AttendanceMsg()
            msg.student_id = student_id
            msg.seat_id = seat_id
            msg.status = status
            msg.timestamp = time.time()
            self.publisher_.publish(msg)
            self.get_logger().info(f'등록 완료: 학번={student_id}, 좌석={seat_id}, 상태={status}')


def main(args=None):
    rclpy.init(args=args)
    node = AttendanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()