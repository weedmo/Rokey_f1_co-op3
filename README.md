### 🔧 GAZEBO_PLUGIN_PATH 설정

`GAZEBO_PLUGIN_PATH` 환경 변수를 설정하여 Gazebo가 사용자 정의 플러그인을 인식하도록 합니다.

1. `{your_ws}`를 본인의 워크스페이스 이름으로 바꿔주세요 (예: `team_f_01_ws`).

2. 다음 명령어를 `.bashrc`에 추가하고 적용하세요:

   ```bash
   echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/your_ws/src/turtlebot3_manipulation/turtlebot3_manipulation_bringup/worlds >> ~/.bashrc
   source ~/.bashrc
