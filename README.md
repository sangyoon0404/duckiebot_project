# Isaac Sim 기반 Duckiebot 자율주행 프로젝트

## 1. 프로젝트 개요 (Project Overview)
이 프로젝트는 **NVIDIA Isaac Sim** 가상 환경에서 **Duckiebot**의 자율주행(Red Box 추적)을 구현한 결과물입니다.
**ROS2 Humble**과 **Python**을 사용하여 **고수준(High-level)** 및 **저수준(Low-level)** 제어 시스템을 통합 구현하였습니다.

* **개발 환경:** WSL2 (Ubuntu 22.04), ROS2 Humble, NVIDIA Isaac Sim 4.x
* **주요 기능:**
    * OpenCV 기반 색상 인식 및 추적 (Red Box Follower)
    * P-Control 기반 자율 주행 알고리즘
    * 역운동학(Inverse Kinematics) 기반 모터 제어

## 2. 파일 설명 (Files)
* **`lsy_project.usd` (Simulation Stage)**
  - **[핵심]** NVIDIA Isaac Sim 프로젝트 파일입니다.
  - Duckiebot 로봇 모델, 물리 속성(Physics), 카메라 센서 설정이 포함되어 있습니다.
  - **Action Graph**가 내장되어 있어 ROS2와 시뮬레이터 간의 통신을 담당합니다.

* **`duckie_driver.py` (High-level Control)**
  - 카메라 영상 토픽(`/duckie/camera/image_raw`)을 구독하여 빨간색 물체를 인식합니다.
  - 물체와의 거리(면적)에 따라 속도를 조절하고, 위치 오차에 따라 회전을 제어합니다.
  - `/duckie/cmd_vel` 토픽으로 Twist(선속도, 각속도) 명령을 발행합니다.

* **`duckie_low_level.py` (Low-level Control)**
  - `cmd_vel` 명령을 구독하여 역운동학(Inverse Kinematics)을 계산합니다.
  - 로봇의 기구학적 정보(L, R)를 반영하여 좌/우 바퀴의 회전 속도(rad/s)로 변환합니다.
  - `/duckie/wheel_left`, `/duckie/wheel_right` 토픽으로 최종 모터 명령을 발행합니다.

## 3. 실행 방법 (How to Run)

이 프로젝트는 **WSL2 (Ubuntu 22.04)** 터미널과 **Windows상의 Isaac Sim**을 연동하여 실행합니다.

### 1단계: 시뮬레이션 환경 실행 (Isaac Sim)
1. Windows에서 **NVIDIA Isaac Sim**을 실행합니다.
2. 상단 메뉴에서 `File` -> `Open`을 클릭합니다.
3. 저장소에 포함된 **`lsy_project.usd`** 파일을 선택하여 엽니다.
4. 로딩이 완료되면 왼쪽 도구 모음의 **[Play]** (재생) 버튼을 반드시 눌러주세요.
   * *(Play를 눌러야 ROS2 Bridge가 활성화됩니다.)*

### 2단계: 저수준 제어 노드 실행 (Terminal 1)
새로운 WSL 터미널을 열고 아래 명령어를 입력합니다.
```bash
# ROS2 환경 설정 (필수)
source /opt/ros/humble/setup.bash

# 프로젝트 폴더로 이동 (경로는 본인 환경에 맞게 수정)
cd ~/path/to/your/project

# 코드 실행
python3 duckie_low_level.py
3단계: 자율주행 드라이버 실행 (Terminal 2)
또 다른 WSL 터미널을 열고 실행합니다.

Bash

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 프로젝트 폴더로 이동
cd ~/path/to/your/project

# 코드 실행
python3 duckie_driver.py
4단계: 실행 확인
OpenCV 창: "Red Box View" 창이 뜨고 로봇의 시야(초록색 박스)가 보입니다.

Isaac Sim: 로봇이 빨간 박스를 따라 스스로 움직이는 것을 확인합니다.

터미널 로그: Target Reached! Stopping 등의 메시지가 출력되는지 확인합니다.
