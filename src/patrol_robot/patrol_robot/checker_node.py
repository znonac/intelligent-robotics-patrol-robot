"""
착석 감지 노드 (checker_node)

YOLOv8 pre-trained 모델(COCO person 클래스)로 사람을 감지하고,
bounding box와 좌석 ROI의 겹침 비율(intersection/bbox area)이
roi_overlap_threshold 이상일 때만 착석으로 판정한다.

흐름:
  /patrol_target 수신 → detection_window_sec 동안 프레임 수집
  → YOLOv8 person 감지 + ROI 겹침 비율 검사 → /seat_occupied 발행

설치 필요: pip install ultralytics
"""

import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from attendance_msgs.msg import SeatTarget, DetectionResult


class CheckerNode(Node):

    def __init__(self):
        super().__init__('checker_node')

        self.declare_parameter('detection_window_sec', 3.0)
        self.declare_parameter('detection_threshold', 0.4)   # 40% 이상 프레임 감지 시 착석
        self.declare_parameter('min_frames', 3)
        self.declare_parameter('confidence_score', 0.2)      # YOLO confidence threshold
        # ROI: 착석 판정 영역 (화면 비율, 0.0~1.0)
        self.declare_parameter('roi_x_min', 0.15)
        self.declare_parameter('roi_x_max', 0.85)
        self.declare_parameter('roi_y_min', 0.25)
        self.declare_parameter('roi_y_max', 1.00)
        # bbox가 ROI에 이 비율 이상 겹쳐야 착석으로 판정 (인접 좌석 오감지 방지)
        self.declare_parameter('roi_overlap_threshold', 0.5)

        self.window_sec = self.get_parameter('detection_window_sec').value
        self.threshold = self.get_parameter('detection_threshold').value
        self.min_frames = self.get_parameter('min_frames').value
        self.conf_score = self.get_parameter('confidence_score').value
        self.roi = {
            'x_min': self.get_parameter('roi_x_min').value,
            'x_max': self.get_parameter('roi_x_max').value,
            'y_min': self.get_parameter('roi_y_min').value,
            'y_max': self.get_parameter('roi_y_max').value,
        }
        self.roi_overlap_thresh = self.get_parameter('roi_overlap_threshold').value

        self.bridge = CvBridge()
        self.model = self._load_yolo()

        self.current_target: SeatTarget | None = None
        self.frames: list = []
        self.active = False
        self.window_start = None

        self.create_subscription(SeatTarget, '/patrol_target', self._on_target, 10)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)

        self.result_pub = self.create_publisher(DetectionResult, '/seat_occupied', 10)

        self.create_timer(0.1, self._check_window)

        self.get_logger().info('착석 감지 노드 준비 완료 (YOLOv8 + ROI overlap)')

    # ── YOLOv8 로드 ───────────────────────────────────────────────────────────

    def _load_yolo(self):
        try:
            from ultralytics import YOLO
            model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8 모델 로드 완료')
            return model
        except ImportError:
            self.get_logger().error(
                'ultralytics 미설치. pip install ultralytics 실행 후 재시작하세요.')
            return None

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _on_target(self, msg: SeatTarget):
        self.current_target = msg
        self.frames = []
        self.active = True
        self.window_start = self.get_clock().now()
        self.get_logger().info(
            f'감지 시작: 좌석={msg.seat_id}, {msg.patrol_round}차 순찰 '
            f'({self.window_sec:.0f}초 윈도우)')

    def _on_image(self, msg: Image):
        if not self.active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frames.append(frame)
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')

    def _check_window(self):
        if not self.active or self.window_start is None:
            return
        elapsed = (self.get_clock().now() - self.window_start).nanoseconds / 1e9
        if elapsed >= self.window_sec:
            self._finalize_detection()

    # ── 감지 분석 ─────────────────────────────────────────────────────────────

    def _finalize_detection(self):
        self.active = False
        target = self.current_target

        if self.model is None:
            self.get_logger().error('YOLOv8 모델 없음 — 미감지로 처리')
            self._publish_result(target, False, 0.0)
            return

        if len(self.frames) < self.min_frames:
            self.get_logger().warn(f'프레임 부족 ({len(self.frames)}개)')
            self._publish_result(target, False, 0.0)
            return

        detected_frames = 0
        for frame in self.frames:
            if self._detect_person_with_roi(frame):
                detected_frames += 1

        confidence = detected_frames / len(self.frames)
        person_detected = confidence >= self.threshold

        flag = '✓ 착석' if person_detected else '✗ 미착석'
        self.get_logger().info(
            f'감지 완료: {target.seat_id} — {flag} '
            f'({detected_frames}/{len(self.frames)}프레임, 신뢰도 {confidence:.0%})')

        self._publish_result(target, person_detected, confidence)

    def _detect_person_with_roi(self, frame) -> bool:
        """YOLOv8 person 감지 + ROI 겹침 비율 검사.

        bbox와 좌석 ROI의 intersection/bbox_area >= roi_overlap_threshold 일 때만
        착석으로 판정한다. 인접 좌석이나 지나가는 사람으로 인한 오감지를 방지한다.
        """
        h, w = frame.shape[:2]

        rx1 = self.roi['x_min'] * w
        rx2 = self.roi['x_max'] * w
        ry1 = self.roi['y_min'] * h
        ry2 = self.roi['y_max'] * h

        results = self.model(
            frame,
            classes=[0],          # person 클래스만
            conf=self.conf_score,
            verbose=False,
        )

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                ix1 = max(x1, rx1)
                iy1 = max(y1, ry1)
                ix2 = min(x2, rx2)
                iy2 = min(y2, ry2)

                if ix2 <= ix1 or iy2 <= iy1:
                    continue

                intersection = (ix2 - ix1) * (iy2 - iy1)
                bbox_area = (x2 - x1) * (y2 - y1)

                if bbox_area > 0 and intersection / bbox_area >= self.roi_overlap_thresh:
                    return True

        return False

    def _publish_result(self, target: SeatTarget, detected: bool, confidence: float):
        msg = DetectionResult()
        msg.seat_id = target.seat_id
        msg.student_id = target.student_id
        msg.person_detected = detected
        msg.confidence = confidence
        msg.patrol_round = target.patrol_round
        msg.stamp = self.get_clock().now().to_msg()
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
