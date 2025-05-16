# 🧠 TurtleBot3 Manipulation + Gazebo 시뮬레이션 패키지

이 프로젝트는 ROS 2 Humble 기반에서 TurtleBot3 Manipulation 시뮬레이션을 위한 환경을 제공합니다.

---

## 📦 사전 요구 사항

### ✅ 1. ROS 2 Humble 설치

Ubuntu 22.04 기준 ROS 2 Humble 설치:

👉 [ROS 2 Humble 설치 가이드](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### ✅ 2. TurtleBot3 e-Manual 설정

👉 [TurtleBot3 e-Manual - ROS 2 Humble](https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/)

> 참고: 모델은 `waffle_pi` 기준으로 설정됩니다. `burger` 또는 `waffle` 사용 시 적절히 수정해주세요.

---

## 🔧 환경 설정

### 1. 워크스페이스 생성 및 패키지 복사

```bash
mkdir -p ~/team_f_01_ws/src
cd ~/team_f_01_ws/src
git clone https://github.com/weedmo/Rokey_f1_co-op3.git
```

### 2. 종속성 설치

```bash
cd ~/team_f_01_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 빌드

```bash
colcon build --symlink-install
```

### 4. 환경 변수 설정

`.bashrc`에 다음 내용을 추가하고 적용합니다:

```bash
# TurtleBot3 모델 설정 (예: waffle_pi)
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc

# 사용자 정의 Gazebo 모델 및 플러그인 경로 설정
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/team_f_01_ws/src/Rokey_f1_co-op3/turtlebot3_manipulation_bringup/worlds' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$HOME/team_f_01_ws/build/turtlebot3_gazebo:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc

# 워크스페이스 overlay
echo 'source ~/team_f_01_ws/install/setup.bash' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
# 적용
source ~/.bashrc
```

---

## 🚀 실행 예시

```bash
# 새 터미널에서 실행 (환경 적용 후)
ros2 launch turtlebot3_manipulation_moveit_config moveit_gazebo.launch.py 

# 새 터미널에서 실행
ros2 launch turtlebot3_rokey_camera line_tracing.launch.py 

```

---

## 📎 참고

- Gazebo에서 카메라/라이다 등의 센서 시뮬레이션을 위한 플러그인이 정상적으로 작동하지 않을 경우, `gazebo_ros_pkgs` 설치 여부 확인
- 시뮬레이션 화면이 검게 나오는 경우: GPU 드라이버 및 Gazebo 환경 확인
