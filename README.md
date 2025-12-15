# Isaac Sim 기반 Duckiebot 자율주행 프로젝트

## 1. 프로젝트 개요
이 프로젝트는 NVIDIA Isaac Sim 환경에서 Duckiebot의 자율주행(Red Box 추적)을 구현한 코드입니다.
ROS2 Humble과 Python을 사용하여 고수준/저수준 제어 시스템을 통합하였습니다.

## 2. 파일 설명
* **`duckie_driver.py`**: 
  - 카메라 영상을 받아 빨간색 물체를 인식합니다. (OpenCV)
  - 물체와의 거리에 따라 속도를 조절하는 P-Control 로직이 들어있습니다.
  - `/duckie/cmd_vel` 토픽으로 Twist(속도, 회전) 명령을 보냅니다.

* **`duckie_low_level.py`**:
  - `cmd_vel` 명령을 받아 역운동학(Inverse Kinematics)을 계산합니다.
  - 좌/우 바퀴의 회전 속도(rad/s)로 변환하여 모터를 구동합니다.

## 3. 실행 방법
```bash
# 1. 저수준 제어 노드 실행
python3 duckie_low_level.py

# 2. 자율주행 드라이버 실행
python3 duckie_driver.py
