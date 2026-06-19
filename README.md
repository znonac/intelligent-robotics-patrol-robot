# 좌석 기반 대리출석 감지 순찰 로봇

> Ewha Womans University — Intelligent Robotics Final Project

---

## 팀 정보

| 항목 | 내용 |
|------|------|
| 팀명 | 딸기라떼 |
| 팀원 | 강지호 (2476003) |
| 과목 | 지능형 로보틱스 (송다은 교수님) |

---

## 프로젝트 설명

강의실 환경에서 TurtleBot3 Waffle 로봇이 Nav2를 이용해 자율 주행하며, YOLOv8 비전 모델로 각 좌석의 착석 여부를 감지하는 **출석 관리 시스템**입니다.

학생이 학번과 좌석번호를 등록하면 로봇이 순찰을 시작하고, 최대 3회 순찰을 통해 출석·지각·대리출석 의심·결석을 자동 판정합니다. 결과는 CSV 파일로 저장되고, RViz2에서 색상 마커로 실시간 시각화됩니다.

---

## 시스템 구조 (ROS2 Node 구성)

```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 System                          │
│                                                             │
│  ┌──────────────────┐   /attendance    ┌────────────────┐   │
│  │  attendance_node │ ──────────────► │   patrol_node  │   │
│  │  (출석 등록 CLI)  │                  │  (자율 순찰 +  │   │
│  └──────────────────┘                  │   출석 판정)   │   │
│                                        └───────┬────────┘   │
│  /patrol_target (SeatTarget)                   │            │
│  ◄─────────────────────────────────────────────┘            │
│  │                                             ▲            │
│  ▼                                             │            │
│  ┌──────────────────┐  /seat_occupied          │            │
│  │  checker_node    │ ────────────────────────►│            │
│  │  (YOLOv8 착석    │  (DetectionResult)        │            │
│  │   감지)          │                           │            │
│  └──────────────────┘                           │            │
│                                                 │            │
│  /attendance_status (PatrolReport)              │            │
│  ◄──────────────────────────────────────────────┘            │
│  │                                                           │
│  ▼                                                           │
│  ┌──────────────────┐                                        │
│  │ visualizer_node  │ ──► /attendance_markers ──► RViz2     │
│  │  (RViz2 마커     │                                        │
│  │   시각화)        │                                        │
│  └──────────────────┘                                        │
└─────────────────────────────────────────────────────────────┘
```

### 패키지 구성

| 패키지 | 노드 | 역할 |
|--------|------|------|
| `attendance_msgs` | — | 커스텀 메시지 정의 (4종) |
| `attendance_robot` | `attendance_node` | 학번·좌석번호 CLI 입력 후 `/attendance` 발행 |
| `patrol_robot` | `patrol_node` | Nav2 자율 주행 + 출석 판정 + CSV 저장 |
| `patrol_robot` | `checker_node` | YOLOv8 person 감지 + ROI 겹침 비율 필터링 |
| `patrol_robot` | `visualizer_node` | RViz2 MarkerArray 출석 상태 시각화 |
| `patrol_robot` | `initial_pose_node` | AMCL 초기 위치 자동 발행 |
| `gazebo_simulation` | — | Gazebo 강의실 월드 및 지도 |

### 커스텀 메시지

| 메시지 | 필드 |
|--------|------|
| `AttendanceMsg` | seat_id, student_id, status, timestamp |
| `SeatTarget` | seat_id, student_id, pos_x, pos_y, pos_yaw, patrol_round |
| `DetectionResult` | seat_id, student_id, person_detected, confidence, patrol_round, stamp |
| `PatrolReport` | student_id, seat_id, total_patrols, detected_count, final_status, stamp |

### 토픽 / 서비스

| 이름 | 타입 | 방향 |
|------|------|------|
| `/attendance` | `AttendanceMsg` | attendance_node → patrol_node |
| `/patrol_target` | `SeatTarget` | patrol_node → checker_node |
| `/seat_occupied` | `DetectionResult` | checker_node → patrol_node |
| `/attendance_status` | `PatrolReport` | patrol_node → visualizer_node |
| `/attendance_markers` | `MarkerArray` | visualizer_node → RViz2 |
| `/start_patrol` | `std_srvs/Trigger` | 서비스 (순찰 시작 트리거) |
| `/initialpose` | `PoseWithCovarianceStamped` | initial_pose_node → AMCL |

---

## 구현 방법

### 순찰 로직 (patrol_node)

1. **1차 순찰**: 등록된 전체 좌석을 순찰 (지각 학생 좌석 우선)
2. **2차 순찰**: 1차에서 미감지된 좌석만 재확인 (5분 대기)
3. **3차 순찰**: 2차에서도 미감지된 좌석 최종 확인 (5분 대기)

- Nav2 실패 시 3초 대기 후 최대 3회 자동 재시도, 초과 시 스킵
- 감지 응답이 없을 경우 watchdog 타임아웃으로 자동 처리
- 순찰 결과(회차별 착석 여부 + 신뢰도)를 CSV로 자동 저장

> **버그 수정 사례**: `_build_queue()` 의 `results.get((s, prev), True)` 를 `False` 로 수정.  
> Nav 실패로 인해 `results` 에 기록이 없는 좌석이 기본값 `True` 로 인해 "착석 확인됨"으로  
> 잘못 처리되어 재확인 대상에서 누락되는 엣지 케이스 → FPR 개선.

### 출석 판정 기준

| 판정 | 조건 | RViz 색상 |
|------|------|-----------|
| 출석 (present) | 착석 감지 + 등록 시각 10분 이내 | 초록 |
| 지각 (late) | 착석 감지 + 등록 시각 10~30분 | 노랑 |
| 대리출석 의심 (suspicious) | 등록했으나 모든 순찰에서 미감지 | 빨강 |
| 결석 (absent) | 미등록 좌석 | 회색 |

### 착석 감지 (checker_node)

- YOLOv8n (COCO person 클래스 id=0) 모델 사용
- 3초 감지 윈도우 동안 수집된 프레임의 40% 이상에서 사람 감지 시 착석 판정
- **ROI 겹침 비율 방식**: bounding box와 좌석 ROI의 `intersection / bbox_area ≥ 0.5` 조건  
  → 인접 좌석의 학생이나 지나가는 사람으로 인한 오감지 최소화

### 시각화 (visualizer_node)

- RViz2 MarkerArray: 좌석 위치에 색상 구체 + 학번·상태 텍스트
- 마커 lifetime=2초 설정 → 노드 종료 시 RViz2에서 자동 소멸

---

## 실행 방법

### 사전 설치

```bash
pip install ultralytics
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 실행 순서

**터미널 1** — Gazebo 강의실 시뮬레이션

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch gazebo_simulation classroom.launch.py
```

**터미널 2** — Nav2 자율 주행

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=$(ros2 pkg prefix gazebo_simulation)/share/gazebo_simulation/maps/classroom.yaml
```

**터미널 3** — 출석 관리 시스템 (노드 일괄 실행 + 초기 위치 자동 설정)

```bash
ros2 launch patrol_robot patrol_system.launch.py
```

**터미널 4** — 학생 출석 등록 후 순찰 시작

```bash
# attendance_node 터미널에서 학번/좌석 입력 후:
ros2 service call /start_patrol std_srvs/srv/Trigger
```

출석 로그: `~/ros2_ws/attendance_logs/attendance_YYYYMMDD_HHMMSS.csv`

---

## AI 사용 내용

Claude AI (Anthropic)를 활용하여 아래 내용을 지원받았습니다:

- ROS2 노드 4종(attendance_node, patrol_node, checker_node, visualizer_node) 구조 설계 및 코드 구현
- patrol_node의 다회 순찰 큐 로직, Nav2 ActionClient 비동기 콜백 구조
- checker_node의 YOLOv8 ROI 겹침 비율(intersection/bbox area) 게이팅 로직
- visualizer_node의 RViz2 MarkerArray TEXT_VIEW_FACING 마커 구현
- Gazebo Harmonic SDF 월드 파일 (벽·책상·Gazebo Actor 동적 학생 모델)
- `_build_queue()` 버그 발견 및 수정 (`True → False` FPR 개선)
- AMCL 초기 위치 자동 발행 노드(initial_pose_node) 설계

핵심 로직 설계 방향 결정, 파라미터 튜닝, 실제 빌드·실행 테스트, 데모 영상 촬영은 직접 수행하였습니다.

---

## 참고 자료

- [Ultralytics YOLOv8 공식 문서](https://docs.ultralytics.com)
- [ROS2 Nav2 공식 문서](https://docs.nav2.org)
- [Gazebo Harmonic SDF 레퍼런스](https://gazebosim.org/docs/harmonic/sdf)
- [Gazebo Fuel Models (standing/walking person)](https://app.gazebosim.org/fuel/models)
- [TurtleBot3 공식 매뉴얼](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [ROS2 Jazzy 공식 문서](https://docs.ros.org/en/jazzy/)
- 이화여자대학교 지능형 로보틱스 강의 자료 (송다은 교수님)

---

## 데모 영상

- [https://youtu.be/al8cwplNHZ0?si=SNZjexcQbMGGz-mJ]

## GitHub

- [https://github.com/znonac/intelligent-robotics-patrol-robot]

---

## 환경 정보

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| Gazebo | Harmonic |
| Python | 3.12 |
| YOLOv8 | ultralytics 8.x (yolov8n.pt) |
| 로봇 | TurtleBot3 Waffle |
