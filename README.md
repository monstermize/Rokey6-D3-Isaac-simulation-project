# Rokey6-D3-Isaac-simulation-project

# 🚗 Isaac Sim & ROS 2 Autonomous Navigation

## 📖 프로젝트 소개
이 프로젝트는 **NVIDIA Isaac Sim** 가상 환경에서 작동하는 자율주행 시뮬레이션입니다. **ROS 2 (Humble)** 통신을 기반으로, A* 알고리즘을 이용한 최단 경로 탐색과 카메라 비전(OpenCV)을 활용한 실시간 차선 유지 자율주행 기능을 구현했습니다.

## 📂 프로젝트 구조 (Directory Structure)
프로젝트의 주요 폴더 및 파일 구성은 다음과 같습니다.

    src/project/
    ├── project/
    │   ├── map_car.py                 # Isaac Sim 환경 및 차량 소환 노드
    │   └── line_detecing.py           # 자율주행 알고리즘 및 제어 노드
    └── resource/
        ├── assets/                    # 맵 및 모델링에 필요한 추가 에셋 폴더
        ├── ackermann_car_fixed_cam.usd # 카메라가 부착된 자율주행 차량 모델 파일
        └── map.usd                    # 가상 주행 환경 맵 파일

## ✨ 주요 기능
* **Isaac Sim & ROS 2 연동:** 시뮬레이터와 ROS 2 브릿지를 연결하여 실시간 데이터(이미지, Odometry, 제어 명령) 통신.
* **A* 알고리즘 경로 탐색:** 맵 상의 노드와 교차로를 그래프로 구성하여 목적지까지의 최단 경로 생성.
* **차선 인식 (Vision 모드):** 영상에서 HSV 색상 필터링으로 파란색 차선을 인식하고 오차에 따라 부드럽게 핸들 조향.
* **교차로 횡단 (Blind 모드):** 차선이 없는 교차로 진입 시 목표 지점의 각도를 계산하여 지정된 조향각으로 안전하게 통과.
* **실시간 맵 UI 제공:** Matplotlib과 NetworkX를 사용하여 현재 차량 위치와 남은 경로를 화면에 시각적으로 표시.

## 🛠 요구 사항 (Prerequisites)
코드를 실행하기 전 아래 환경이 구성되어 있어야 합니다.
* Ubuntu 22.04
* ROS 2 Humble
* NVIDIA Isaac Sim 5.0
* Python 주요 패키지: rclpy, opencv-python, numpy, matplotlib, networkx

## 🚀 실행 방법 (Usage)

이 프로젝트는 2개의 터미널을 열어 각각 시뮬레이터와 자율주행 노드를 실행해야 합니다.

### Terminal 1: Isaac Sim 시뮬레이터 및 맵 환경 실행
환경 변수를 설정하여 ROS 2 통신 브릿지를 열고, `resource` 폴더에 있는 맵과 차량을 소환합니다.

    export ROS_DISTRO=humble
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/humble/lib

    /home/rokey/isaacsim/python.sh /home/rokey/IsaacSim-ros_workspaces/humble_ws/src/project/project/map_car.py

### Terminal 2: 자율주행 알고리즘 노드 실행
새 터미널을 열고 자율주행 코드를 실행합니다. 스크립트가 실행되면 팝업되는 UI 창에 이동할 목적지를 입력해주세요 (예: fire_station, home 등).

    source /opt/ros/humble/setup.bash
    cd ~/IsaacSim-ros_workspaces/humble_ws/src/project/project
    python3 line_detecing.py

## 📝 파일 상세 설명

* **`map_car.py`**: 가상 세계를 만들고 환경을 띄우는 베이스 파일입니다. ROS 2와 통신할 수 있는 통로를 열고, `resource` 폴더에서 `map.usd`를 가져와 바닥에 깔고 그 위에 `ackermann_car_fixed_cam.usd` 차량을 떨어지지 않게 소환합니다.
* **`line_detecing.py`**: 차량의 두뇌 역할을 담당하며 목적지까지 차를 스스로 운전하게 만듭니다. 지도를 그려 A* 알고리즘으로 최단 거리를 찾고, 카메라로 차선을 보며 똑바로 달리게 합니다. 차선이 없는 교차로에서는 지도상의 각도를 계산해 스스로 핸들을 꺾습니다.
* **`resource/` 폴더**: 화면에 보여질 3D 모델링 파일들이 모여있는 창고입니다. 차량 외형, 카메라 위치, 맵의 건물과 도로 정보가 모두 이 안의 `.usd` 파일과 `assets` 폴더에 저장되어 있습니다.
