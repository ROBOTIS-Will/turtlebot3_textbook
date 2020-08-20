---
layout: splash
lang: kr
ref: turtlebot3_textbook3-2
permalink: /docs/turtlebot3_textbook/week3-2/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook3-2"
---

# TurtleBot3 Manipulator를 사용하여  SLAM 실행하기
OpenMANIPULATOR-X가 장착된 TurtleBot3의 SLAM은 앞에서 학습했던 SLAM과 조금 차이가 있습니다.
로봇 팔이 LDS 센서의 일정 부분을 가로막고 있기 때문에 SLAM에 사용되는 LDS센서의 범위를 제한해주어야 원활한 지도의 작성이 가능해집니다.  
아래와 같이 `turtlebot3_manipulation_slam/config/scan_data_filter.yaml` 파일에서 LDS 센서의 설정된 각도 범위 데이터를 필터링함으로써 유효하지 않은 각도의 값을 사용하지 않고 지도를 그릴 수 있습니다.

```
scan_filter_chain:

- name: Remove 120 to 240 degree
  type: laser_filters/LaserScanAngularBoundsFilterInPlace
  params:
    lower_angle: 2.0944
    upper_angle: 4.18879
```

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_slam.png)

## roscore 실행하기
[Remote PC] 아래 명령어로 roscore를 실행합니다.
```bash
$ roscore
```

## BringupBringup 실행하기
[TurtleBot3 SBC] 아래 명령어로 rosserial과 LDS센서를 동작시키는 노드를 실행합니다.
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
{% capture capture01 %}
**roslaunch turtlebot3_bringup turtlebot3_robot.launch**
1. **turtlebot3_core.launch**
    - subscribe : cmd_vel, joint_trajectory_point, gripper_position
    - publish : joint_states, odom

2. **turtlebot3_lidar.launch**
    - publish : scan

OpenCR 펌웨어가 변경되었기 때문에 turtlebot3_robot.launch 파일을 실행시키면 앞서 설명한 turtlebot3_robot.launch를 실행시켰을 때 생성되는 토픽들 이외에 joint_trajectory_point, gripper_position 두가지 토픽을 subscribe합니다. joint_trajectory_point는 로봇팔의 각 조인트의 위치값이며 OpenCR을 통해 로봇팔의 각 모터에 전달됩니다. gripper_position은 로봇 그리퍼의 조인트 위치값이며 함께 OpenCR을 통해 로봇 그리퍼 모터에 전달되어 로봇이 움직이게 됩니다. rqt의 Message Publisher를 사용하여 위치값을 주어 간단히 제어할 수 있습니다.
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>

SLAM을 사용하여 지도를 작성할 때에는 Manipulation을 사용하지 않기 때문에 아래의 OpenMANIPULATOR를 제어하는 컨트롤러와 move_group 인터페이스는 실행할 필요가 없습니다.

```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

## SLAM 노드 실행하기
[Remote PC] 여기에서는 Gmapping을 활용한 SLAM을 실행합니다.

```bash
$ roslaunch turtlebot3_manipulation_slam slam.launch
```
{% capture capture06 %}
**roslaunch turtlebot3_manipulation_slam slam.launch**
1. **urdf**
  - Unified Robot Description Format의 약자로써 로봇의 구성과 연결형태를 표현하는 XML 형식의 파일입니다.
2. **robot_state_publisher**
  - robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf의 형식으로 publish 합니다.
  - subscribe : joint_states 
  - publish : tf
3. **laser_filter노드**
  - LDS센서의 유효하지 않은 값의 영역을 필터링하는 노드를 실행합니다. 여기에서는 OpenMANIPULATOR가 설치되어 있는 후방부에 대한 각도를 무시합니다.
4. **turtlebot3_gmapping.launch**
  - Gmapping을 이용한 SLAM을 실행하기 위해 필요한 파라미터 정보들이 파라미터 서버에 로딩됩니다. 이 설정을 이용해 gmapping을 설정합니다.
5. **turtlebot3_gmapping.rviz**
  - Gmapping을 적용한 SLAM을 RViz 화면에 표시하기 위해 필요한 RViz의 기본 설정을 적용하여 RViz를 실행시킵니다.

turtlebot3_manipulation_slam.launch 파일을 실행하면 로봇의 urdf가 정의된 위치에서 불려옵니다. 또한 joint_states과 urdf을 이용하여 tf을 publish하는 robot_state_publisher 노드를 생성합니다. 
{% endcapture %}
<div class="notice--success">{{ capture06 | markdownify }}</div>

## turtlebot3_teleop_key 노드 실행하기
[Remote PC] 작성되지 않은 지도의 위치로 로봇을 이동시켜 지도를 완성합니다.
```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

[Remote PC] 지도가 완성되고 나면 map_saver 노드를 실행시켜 지도파일을 저장합니다.
```bash
$ rosrun map_server map_saver -f ~/${map_name}
```
<-f> 옵션은 지도 파일을 저장할 위치와 파일 이름을 지정합니다. 위 명령은 ~/${map_name} 옵션을 사용하고 있기 때문에, 사용자의 home 폴더 (~/ 또는 /home/<username>)에 `${map_name}.pgm`와 `${map_name}.yaml` 파일로 저장됩니다. ${map_name}에 저장할 파일 이름을 입력하십시오.

# Navigation
OpenMANIPULATOR를 장착한 TurtleBot3의 Navigation은 기본 TurtleBot3의 플랫폼에서 실행하는 Navigation과 크게 다르지 않습니다. 다만 SLAM에서와 마찬가지로 LDS 센서의 범위를 지정해 주는 것이 좋으며, Navigation 도중 필요한 경우 OpenMANIPULATOR를 구동할 수 있도록 로봇팔과 그리퍼를 제어할 수 있는 관련 노드를 실행해 줄 수 있습니다.

## roscore 실행하기
[Remote PC] roscore를 실행합니다.
```bash
$ roscore
```

## Bringup 실행하기
[TurtleBot3 SBC] 아래 명령어로 rosserial과 LDS센서를 동작시키는 노드를 실행합니다.
```bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
<div class="notice--success">{{ capture01 | markdownify }}</div>

## Navigation 실행하기
[Remote PC] 아래의 명령어를 실행하면 Navigation을 실행하기 위해 필요한 여러가지 파라미터와 지도, GUI 환경을 만들기 위한 URDF와 RViz 환경설정 등을 불러들입니다. 많은 노드들이 동시에 실행되는 실행파일이므로 실행되는 파일과 노드를 먼저 확인하고 실행하시기 바랍니다.

```bash
$ roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml
```
{% capture capture07 %}
**roslaunch turtlebot3_manipulation_navigation navigation.launch map_file:=~/${map_name}.yaml**
1. **urdf**
  - TurtleBot3와 OpenMANIPULATOR가 결합된 형태의 turtlebot3_manipulation_robot.urdf.xacro 파일을 읽어들입니다. 이 파일에서는 TurtleBot3의 형태를 기술한 파일과 OpenMANIPULATOR의 형태를 기술한 파일을 결합하여 전체적인 로봇의 형태를 만들어냅니다. 
2. **robot_state_publisher**
  - robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf의 형식으로 publish 합니다.
3. **laser_filter**
  - LDS 센서의 설정된 각도 범위 데이터를 필터링합니다.
4. **map_server**
  - SLAM으로 완성된 지도와 설정파일을 읽어들입니다.
5. **amcl.launch**
  - AMCL 파티클 필터를 사용하기 위한 각종 파라미터를 읽어들입니다. 지도 및 센서의 scan 값으로 로봇 initialpose와 tf을 읽고 particle filter를 사용하여 지도에서 로봇의 위치를 예측합니다.
6. **move_base.launch**
  - move_base 노드는 로봇의 Navigation stack에 액세스 할 ROS 인터페이스를 제공합니다. move_base 노드는 global planner와 local planner를 연결하여 로봇을 목적지까지 이동시키며, 이때 각각의 planner에 맞는 costmap도 보관합니다. 로봇 목적지(goal)를 Action 형태로 받으면 현재 위치(feedback)와 상태(status), 이동의 결과(result)를 업데이트하기 위해 마찬가지로 Action 형태로 전달합니다. 또한 현재의 상태에 맞게 로봇을 움직이기 위한 cmd_vel 토픽이 지속적으로 publish됩니다.
7. **rviz**
  - 각종 데이터와 파라미터를 시각화한 GUI 윈도우를 생성합니다.
{% endcapture %}
<div class="notice--success">{{ capture07 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_nav.png)

## OpenMANIPULATOR 제어하기
Navigation을 실행할 때 OpenMANIPULATOR를 제어하는 노드를 생성하면, Navigation과 함께 로봇 팔을 제어할 수 있게 됩니다.  
로봇이 움직이는 동안 OpenMANIPULATOR를 움직이는 경우 진동이나 무게중심의 이동으로 인해 로봇과 로봇팔의 동작이 불안정할 수 있습니다. 로봇 팔은 로봇이 움직이지 않는 상태에서 동작시키는 것을 권장합니다.

### turtlebot3_manipulation_bringup 노드 실행
[Remote PC] OpenMANIPULATOR만을 제어할 때와 마찬가지로 arm_controller와 gripper_controller를 실행합니다.
```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch**

**turtlebot3_manipulation_bringup 노드**

turtlebot3_manipulation_bringup.launch를 실행시키면 arm_controller와 gripper_controller 두 개의 컨트롤러가 실행이 됩니다. move_group과 통신하는 action server 컨트롤러의 역할로써 각각 move_group을 통해 팔과 그리퍼 관절의 목표궤적을 읽어들여 순서대로 publish합니다. publish된 토픽들은 OpenCR을 통해 로봇의 관절에 조립된 DYNAMIXEL에 전달되어 OpenMANIPULATOR를 움직이게 됩니다.
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

### move_group 노드 실행
move_group 노드를 실행한 이후 MoveIt을 사용해서 OpenMANIPULATOR를 제어하거나 ROBOTIS GUI를 이용해 제어할 수 있습니다. 여기에서는 ROBOTIS GUI를 실행하는 방법을 기술합니다. 두 가지 방법 중 적절한 인터페이스를 사용하시기 바랍니다.

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
{% capture capture03 %}
**roslaunch turtlebot3_manipulation_moveit_config move_group.launch**

move_group.launch을 실행하면 move_group노드가 실행됩니다. move_group노드는 유저인터페이스를 통해 명령을 받아와 로봇 컨트롤러에게 action형식으로 전달합니다. 
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

### ROBOTIS GUI 컨트롤러 실행
[Remote PC] 로보티즈 GUI는 OpenMANIPULATOR의 첫번째 DYNAMIXEL을 기준으로 그리퍼의 유효한 파지 위치(그리퍼 사이의 빨간색 육면체)를 레퍼런스로 하는 Task Space Control이나 각 조인트 관절의 각도를 레퍼런스로 하는 Joint Space Control을 지원합니다.  
필요에 따라 편리한 제어 방법을 사용할 수 있습니다.

```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
{% capture capture05 %}
**roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch**

유저인터페이스로 c++ move_group_interface를 사용한 qt gui가 실행됩니다. 인터페이스를 통해 받아온 현재 조인트 위치 및 end-effector의 위치가 gui상에 표시됩니다. Send버튼을 클릭하면 설정된 위치값을 인터페이스를 통해 move_group에게 전달하여 컨트롤러에 전달, 로봇이 움직이게 됩니다.
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>
