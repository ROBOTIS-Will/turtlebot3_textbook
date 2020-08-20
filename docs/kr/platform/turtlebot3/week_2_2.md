---
layout: splash
lang: kr
ref: turtlebot3_textbook2-2
permalink: /docs/turtlebot3_textbook/week2-2/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook2-2"
---

# Navigation
성공적인 내비게이션을 위해서는 costmap의 계산이 필요합니다. costmap은 로봇의 위치와 방향을 표현하는 pose 정보와 센서값, 장애물 정보, SLAM으로 얻어진 지도를 기반으로 계산될 수 있습니다. 계산된 결과는 로봇이 충돌하는 영역, 충돌 가능성이 있는 영역, 자유롭게 이동이 가능한 영역 등으로 표현되며, 이러한 계산결과를 토대로 로봇의 안전한 이동 경로를 생성하는데 도움을 줍니다.  

costmap은 두 종류로 나뉠 수 있는데 로봇의 시작위치에서부터 목적지까지의 경로 계획을 수립하는데 사용되는 global costmap과 로봇의 주변부에서 장애물 등을 회피하기 위한 안전한 경로를 계산하는 local costmap이 있습니다. ROS에서 이러한 costmap은 값으로 표현될 수 있는데, 0은 로봇이 자유롭게 이동할 수 있는 영역이며, 255에 가까워질수록 로봇과 충돌이 발생할 수 있는 영역으로 구분됩니다.

- 0 : 자유영역
- 1 ~ 127 : 낮은 충돌 가능성이 있는 영역
- 128 ~ 252 : 높은 충돌 가능성이 있는 영역
- 253 ~ 254 : 충돌 영역
- 255 : 로봇이 이동할 수 없는 영역

![](http://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png)

> [http://wiki.ros.org/costmap_2d#Inflation](http://wiki.ros.org/costmap_2d#Inflation)

DWA (Dynamic Window Approach)는 장애물을 회피하는 경로를 생성하는데 쓰이는 대표적인 방법입니다. DWA는 속도 공간 내에서 병진 속도(v)와 회전 속도(ω)를 사용해서 로봇이 움직일 수 있는 경로를 예측하고 계산하며, 하드웨어에서 낼 수 있는 로봇의 최대속도가 한계값으로 결정됩니다. global planner는 로봇이 목적지에 도착하도록 따라갈 수 있는 전체 경로를 생성하며 dwa_local_planner는 장애물을 회피하며 global path를 잘 따라가도록 local path를 만듭니다. DWA는 아래와 같은 순서로 동작합니다.

1. 로봇의 제어 위치(dx, dy, dtheta)를 개별적으로 생성합니다.
2. 생성된 각각의 샘플을 로봇의 현재 위치에서부터 시뮬레이션해서 짧은 시간동안 샘플이 적용될 경우 로봇의 위치를 예측합니다.
3. 시뮬레이션의 결과를 점수로 환산합니다. 이때, 장애물과의 거리, 목적지까지의 거리, global plan과의 거리, 속도 등의 요소가 고려되며, 물체와 부딪히는 샘플은 결과 계산에서 배제됩니다.
4. 가장 높은 점수를 획득한 샘플이 선택되어 로봇에 전달됩니다.
5. 1~4 과정을 반복합니다.

## 내비게이션 노드 실행하기
### [Remote PC] roscore 실행하기
  ```bash
  $ roscore
  ```
### [TurtleBot3 SBC] TurtleBot3의 bringup 실행하기
  ```bash
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```
{% capture capture01 %}
**roslaunch turtlebot3_bringup turtlebot3_robot.launch**

1. **turtlebot3_core.launch**
  - subscribe : cmd_vel
  - publish : joint_states, odom
2. **turtlebot3_lidar.launch**
  - publish : scan

turtlebot3_robot.launch 파일을 실행시키면 turtlebot3_core.launch와 turtlebot3_lidar.launch 파일이 실행되며, TurtleBot3의 상태를 체크하는 노드인 turtlebot3_diagnostics가 생성되어 TurtleBot3의 각종 센서 및 하드웨어에 대한 상태정보를 송신(publish) 합니다.  turtlebot3_core.launch 파일에서는 OpenCR과 통신하여 joint_states, odom을 송신(publish)하고 cmd_vel를 수신(subscribe)하는 노드가 생성됩니다. turtlebot3_lidar.launch 파일에서는 LIDAR를 작동시켜 센서로부터 얻어진 scan데이터를 송신(publish)하는 노드가 생성됩니다.
{% endcapture %}
  <div class="notice--success">{{ capture01 | markdownify }}</div>

### [Remote PC] Navigation파일 실행하기

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

{% capture capture02 %}
**roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml**
1. **roslaunch turtlebot3_bringup turtlebot3_remote.launch**
  - urdf：Unified Robot Description Format의 약자로써 로봇의 구성과 연결형태를 표현하는 XML 형식의 파일입니다.
  - robot_state_publisher : robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf의 형식으로 publish 합니다.
    - subscribe : joint_states 
    - publish : tf

    turtlebot3_remote.launch 파일을 실행시키면 로봇의 urdf를 정의된 위치에서 불러옵니다. 또한, joint_states와 urdf를 이용하여 tf를 publish하는 robot_state_publisher 노드를 생성합니다.  
    turtlebot3_slam.launch 파일 내부에 turtlebot3_remote.launch를 포함하기 때문에 turtlebot3_slam.launch가 실행되면 자동적으로 turtlebot3_remote.launch가 가장 먼저 실행됩니다.

2. **map_server노드**
  - publish : map_metadata, map
  - map_server 노드는 디스크에 저장되어있는 지도를 불러오는 역할을 합니다. 터미널에 입력된 명령어에서 map_file 파라미터는 지도의 정보가 저장된 파일의 위치를 전달합니다.

3. **amcl.launch**
  - publish : tf, amcl_pose, particlecloud
  - subscribe : scan, tf, initialpose, map
  - 지도와 센서의 scan값, 로봇의 initialpose와 tf를 읽어들인 다음 particle filter를 사용하여 지도상에서 로봇의 위치를 예측합니다.

4. **move_base**
  - subscribe : goal, cancel
  - publish : feedback, status, result, cmd_vel
  - move_base 패키지의 move_base 노드는 로봇의 Navigation stack에 접근하는 ROS 인터페이스를 제공합니다. move_base 노드는 global planner와 local planner를 연결해서 로봇을 목적지까지 움직이도록 하며, 이때 각각의 planner에 맞는 각각의 costmap 역시 보관합니다. 로봇의 목적지(goal)를 Action 형태의 토픽으로 수신받으면 현재 위치(feedback)와 상태(status), 이동 결과(result)를 업데이트 하기 위해 마찬가지로 Action 형태의 토픽을 사용합니다. 또한, 현재 상태에 맞춰 로봇을 움직이기 위한 cmd_vel 토픽이 지속적으로 publish 됩니다.

5. **rviz**
  - subscribe : tf, odom, map, scan
  - rviz의 설정파일을 적용한 rviz가 실행되어 tf, scan, map을 subscribe하여 데이터를 시각화합니다
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

  위의 명령을 실행하면 시각화 도구 Rviz가 실행됩니다. Rviz를 별도로 실행하기 위해서는 다음의 명령 명령을 사용하십시오.
  ```bash
  $ rviz -d `rospack find turtlebot3_navigation`/rviz/turtlebot3_navigation.rviz
  ```

## 로봇의 초기 Pose 설정하기

Navigation을 할 때 가장 중요한 것은 로봇의 정확한 현재 위치를 지도상에 표현하는 것 입니다. 로봇의 엔코더와 IMU, LDS등 각종 센서로부터 얻어진 정보를 토대로 TurtleBot3의 위치를 예측하는 데에는 확률을 기반으로 하는 파티클 필터가 적용된 AMCL (Adaptive Monte Carlo Localization)이라는 알고리즘이 사용됩니다. AMCL은 센서의 정보를 기반으로 로봇이 존재할 가능성이 있는 위치를 예측하는데 알고리즘의 파라미터 값에 설정된 이동량만큼 로봇이 이동할 때마다 이러한 예측된 위치값을 갱신하며 갱신이 반복될 때마다 예측된 위치값들의 오차가 줄어들게 됩니다.

[**Remote PC**] 로봇이 Navigation을 실행하기 위한 지도가 실행되고 로봇의 LDS 센서가 정상적으로 실행된다면, RViz의 `2D Pose Estimate` 버튼을 사용해서 지도상의 로봇의 위치를 로봇의 실제 위치와 방향에 맞게 설정해주어야 합니다. `2D Pose Estimate` 버튼을 이용해서 초기 위치를 설정할 수 있습니다. LDS 센서의 값과 지도에서 표시된 장애물의 위치가 일치할수록 정확한 Navigation이 가능하므로, 초기 위치를 설정하는 단계를 반복적으로 실행해서 되도록이면 정확한 위치로 설정해야 합니다. 아래의 순서로 로봇의 위치를 설정합니다.

  1. RViz에서 `2D Pose Estimate` 버튼을 클릭합니다 
  2. 지도상에서 로봇이 실제로 위치한 지점을 클릭하고 로봇의 전면이 바라보는 방향으로 화살표를 드래그합니다. 
  3. scan 값과 지도가 어느정도 오버랩될 때까지 위의 과정을 반복합니다.

초기 자세의 설정이 완료되면 로봇은 녹색 화살표로 지정된 위치와 방향을 Pose로 사용하여 실제 로봇의 위치와 방향을 추정합니다. 녹색 화살표는 터틀봇3의 확률적인 위치를 표시합니다. LDS 센서는 지도상에 장애물을 표시합니다. 지도와 LDS 센서의 데이터가 겹쳐질 수 있도록 `2D Pose Estimate` 버튼으로 초기 Pose를 설정해야 합니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_pose_estimate.png)

> 위 그림에서 로봇 주변에 광범위하게 표시된 수많은 작은 초록색 화살표는 로봇의 초기 위치를 지정했을 때, 로봇의 센서와 각종 정보를 통해 AMCL이 계산한 로봇의 현재 위치입니다. 로봇이 이동할 수록 분포되어 있던 현재 위치 예상값들이 로봇쪽으로 가깝게 수렴하는 모습을 볼 수 있습니다.

> **주의** : Navigation을 실행하기 이전에 turtlebot3_teleop_keyboard 노드가 실행되고 있다면 이 노드를 반드시 종료시켜야 합니다. turtlebot3_teleop_keyboard 노드에서 publish되는 cmd_vel 데이터가 Navigation의 cmd_vel 데이터와 충돌하여 로봇이 정상적으로 움직이지 않을 수 있습니다. 

## Navigation Goal 설정하기 
[Remote PC] RViz에서 지도가 표시되고 로봇의 초기 위치가 올바르게 설정되었다면 로봇이 도착할 목적지의 위치와 방향을 지정할 수 있습니다. RViz의 `2D Nav Goal` 버튼을 클릭하고 로봇이 이동할 수 있는 목적지를 클릭하고 로봇이 바라볼 방향을 드래그해서 화살표 방향을 지정합니다.

1. RViz 상단의 `2D Nav Goal` 버튼을 클릭합니다.
2. 지도상에서 로봇이 이동할 목적지를 클릭하고 로봇이 바라볼 방향을 드래그해서 목적시 설정을 완료합니다.

목적지 설정이 완료되면 Navigation 알고리즘은 로봇의 현재 위치에서부터 목적지까지의 경로를 생성하고 경로를 따라갈 수 있도록 로봇에 명령을 내립니다. 로봇이 움직이는 동안 경로상에 장애물이 나타나면 장애물을 회피할 수 있는 경로를 생성해서 이동하게 됩니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/2d_nav_goal.png)

목적지까지의 경로를 생성할 수 없는 경우 Navigation Goal 설정이 실패할 수 있습니다. 로봇이 목적지까지 이동하는 도중에 로봇을 멈추게 하고 싶다면 로봇의 현재 위치를 목적지로 재설정하는 방법을 사용할 수 있습니다.

## 내비게이션 튜닝 가이드
Navigation stack에는 여러 파라미터들이 있으며, 이러한 파라미터의 설정을 통해 서로 다른 형태의 로봇에 최적화된 Navigation을 적용할 수 있습니다. 
여기에서는 중요하거나 자주 사용되는 파라미터에 대한 설명을 합니다. 다양한 로봇과 환경에 따른 더욱 심도있는 Navigation 튜닝은 [Basic Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)를 참고하시기 바랍니다.
아래의 파라미터는 costmap을 계산하는데 사용되는 파라미터이며, `turtlebot3_navigation/param/costmap_common_param_$(model).yaml` 파일에 저장되어 프로그램을 실행할 때 로드됩니다.

### Costmap 관련 파라미터

#### inflation_radius
지도상의 장애물에서 설정된 값만큼의 거리를 설정하여 로봇이 이동하는 경로를 생성할 때 최소한의 안전 거리를 유지하는 데 사용됩니다. 이 값을 로봇의 반경보다 큰 값으로 설정하여 로봇과 장애물 충돌을 피할 수 있습니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_inflation_radius.png)


#### cost_scaling_factor 
지도의 장애물로 표시된 영역의 cost 값의 비중을 조정하는 상수입니다.  
cost_scaling_factor는 다음과 같이 마이너스의 지수 값을 이용하여 파라미터 인자를 생성합니다. cost_scaling_factor 값을 올릴수록 계산된 cost 값은 작아집니다.
costmap_2d :: INSCRIBED_INFLATED_OBSTACLE 값은 254로 정의되어 있습니다.

- exp{-1.0 * cost_scaling_factor * (distance_from_obstacle - inscribed_radius)} * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_cost_scaling_factor.png)


### AMCL 관련 파라미터

#### min_particles, max_particles
AMCL 노드에서 사용하는 파티클 입자의 수를 조정합니다. 입자의 수가 많아지면 정밀도가 높아지지만, 계산량이 증가하기 때문에 연산 속도가 느려집니다.

#### initial_pose_x, initial_pose_y, initial_pose_a
로봇의 초기 위치와 방향을 설정합니다. Rviz의`2D Pose Estimate` 버튼으로 로봇의 Pose를 수동으로 설정할 수도 있습니다.

#### update_min_d, update_min_a
파티클 필터를 업데이트하는 데 필요한 최소 병진거리(m)와 최소 회전각도(rad)를 설정할 수 있습니다. 값이 작을수록 업데이트가 자주 이루어지지만, 계산량이 증가합니다.

### DWA 관련 파라미터
다음은 DWA의 계산에 사용되는 파라미터로써 `turtlebot3_navigation/param/dwa_local_planner_param_$(model).yaml` 파일에 저장되어 프로그램을 실행할 때 로드됩니다.

DWA 관련 파라미터를 로봇의 하드웨어에 맞는 적절한 값으로 설정하지 않는 경우 로봇이 제대로 동작하지 않을 수 있습니다.

#### acc_lim_x, acc_lim_y, acc_lim_th
로봇의 x, y, &theta; 방향으로 가속도의 한계 값을 m/s<sup>2</sup>과 rad/s<sup>2</sup> 단위로 설정합니다.

#### max_trans_vel, min_trans_vel
로봇의 병진 속도의 최대 값과 최소값을 절대 값으로 나타냅니다. 단위는 m/s입니다.

#### max_vel_x, min_vel_x, max_vel_y, min_vel_y
로봇이 x, y 방향으로 이동할 수있는 최대 값, 최소값을 m/s로 설정합니다. 터틀봇3의 경우 y 방향으로의 이동이 불가능하기 때문에, y의 최대, 최소 값은 0으로 설정됩니다.

#### max_rot_vel, min_rot_vel
로봇의 최대, 최소 회전 속도를 rad/s로 설정합니다.

### xy_goal_tolerance 
로봇이 지정된 목적지에 도착했을 때 x, y 좌표상의 거리 오차를 설정합니다.
 
### yaw_goal_tolerance 
로봇이 지정된 목적지에 도착했을 때 로봇이 바라보는 각도에 대한 오차를 설정합니다.
 
### sim_time 
로봇의 현재 위치에서 몇 초만큼의 예상 경로를 시뮬레이션 할 것인지 설정합니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/navigation/tuning_sim_time.png)
