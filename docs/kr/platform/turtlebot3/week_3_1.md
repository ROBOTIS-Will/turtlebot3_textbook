---
layout: splash
lang: kr
ref: turtlebot3_textbook3-1
permalink: /docs/turtlebot3_textbook/week3-1/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook3-1"
---

# [Class 5] Manipulation

OpenMANIPULATOR-X는 ROS를 지원하는 ROBOTIS의 오픈소스 로봇팔입니다. 다이나믹셀과 3D 프린터를 이용해 제작된 부품으로 조립이 가능하기 때문에 만들기 쉽고 가격이 저렴한 장점이 있습니다.
특히 OpenMANIPULATOR-X는 TurtleBot3 Waffle이나 Waffle Pi 버전과 호환이 되도록 설계되었으며, 여기에서는 TurtleBot3 Waffle Pi에 조립된 매니퓰레이터를 사용하는 방법에 대해 알아봅니다. 


## Software 설치
[Remote PC] TurtleBot3에 조립된 OpenMANIPULATOR-X를 사용하기 위해 패키지를 다운로드하고 빌드합니다.

```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
$ sudo apt install ros-kinetic-ros-control && ros-kinetic-ros-controllers && ros-kinetic-control* && ros-kinetic-moveit*
$ cd ~/catkin_ws && catkin_make
```

## Hardware 설정
TurtleBot3 Waffle Pi의 LDS 센서는 로봇의 정 중앙에 위치하고 있습니다.
OpenMANIPULATOR-X를 부착하기 위해서는 LDS 센서의 위치를 아래 그림의 빨간 박스쪽으로 이동시키고, 노란 박스에 OpenMANIPULATOR-X의 첫번째 관절을 부착해야 합니다.
정확한 위치에 부착하지 않으면 로봇의 구성을 설명하는 URDF에 정의된 센서와 로봇 팔의 위치가 실제 로봇의 위치와 달라져 의도하지 않은 동작이 실행되거나 로봇 팔이 의도하지 않은 위치로 이동해 충돌이 생길 수 있습니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble_points.png)

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/assemble.png)

## OpenCR 설정
OpenMANIPULATOR-X가 TurtleBot3 Waffle Pi의 OpenCR에 연결되려면 OpenCR에 연결된 모든 다이나믹셀을 제어할 수 있도록 만들어진 펌웨어를 업로드해야 합니다.

[TurtleBot3 SBC] OpenCR 펌웨어를 TurtleBot3의 Raspberry Pi에 업로드하려면 다음과 같이 입력합니다.
```bash
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=om_with_tb3
$ rm -rf ./opencr_update.tar.bz2
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 && tar -xvf opencr_update.tar.bz2 && cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
```
명령을 입력하고 잠시 후 새로운 펌웨어가 OpenCR에 업로드되고 업로드가 성공하면 터미널 창의 마지막에 `jump_to_fw`라는 문구가 표시됩니다.

{% capture danger01 %}
**경고** : OpenCR의 펌웨어를 성공적으로 업데이트하면 OpenCR이 재부팅되며 OpenMANIPULATOR-X가 기본위치로 움직이게 됩니다. 따라서, OpenMANIPULATOR-X의 전선이 꼬이거나 기본위치로 움직이는 동안 본체나 다른 물체와 부딪히지 않도록 아래와 같은 자세를 만들어준 뒤 펌웨어를 업데이트 하시기 바랍니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/manipulation/open_manipulator_gazebo_1.png)
{% endcapture %}
<div class="notice--danger">{{ danger01 | markdownify }}</div>

## TurtleBot3 Bringup

### roscore 실행하기
[Remote PC] ROS 1을 구동하기 위한 roscore를 사용자의 PC에서 구동시켜줍니다. 
```bash
$ roscore
```

### TurtleBot3 모델 설정
[TurtleBot3 SBC] `.bashrc` 파일에 TURTLEBOT3_MODEL에 대한 정의를 해두지 않았다면, 아래 명령어로 사용중인 TurtleBot3 모델을 정의해주어야 합니다. Manipulation을 위해서는 `waffle_pi` 또는 `waffle` 플랫폼을 사용할 수 있습니다.
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
```

### Bringup 실행하기
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

# Gazebo 시뮬레이터에서 OpenMANIPULATOR 제어하기

## Gazebo 시뮬레이터 실행
[Remote PC] 아래 명령어를 새 터미널창에 입력해서 OpenMANIPULATOR가 적용된 TurtleBot3의 모델을 Gazebo환경으로 불러옵니다.
```bash
$ roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch**

Gazebo상에 OpenMANIPULATOR가 결합된 TurtleBot3 Waffle Pi 모델이 로딩되며, 로봇과 통신하는 두 개의 로봇 컨트롤러인 arm_controller, gripper_controller가 실행됩니다. 각각 로봇의 팔 관절과 그리퍼를 제어하는 컨트롤러입니다. 
방식은 실제 로봇을 사용할 때와 동일합니다.
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_gazebo.png)

## move_group 노드 실행

[Remote PC] MoveIt과 연동하기 위해서는 move_group 노드를 실행시켜 주어야 합니다. Gazebo 시뮬레이터에서 [▶] 실행 버튼을 눌러 시뮬레이션을 시작했다면, 다음 명령어를 입력한 후 아래 그림과 같이 "You can start planning now!" 메세지가 표시됩니다.

```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
{% capture capture03 %}
**roslaunch turtlebot3_manipulation_moveit_config move_group.launch**

move_group.launch을 실행하면 move_group노드가 실행됩니다. move_group노드는 유저인터페이스를 통해 명령을 받아와 로봇 컨트롤러에게 action형식으로 전달합니다. 
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_move_controller.png)

## Rviz 실행하기
[Remote PC] MoveIt 환경이 설정된 `moveit.rviz` 파일을 읽어들여 RViz 상에서 MoveIt을 사용 가능하도록 합니다.
GUI에서 그리퍼에 표시된 Interactive Marker를 활용한 로봇 팔을 제어할 수 있으며, 목표 위치로의 동작을 시뮬레이션 할 수 있기 때문에 충돌 등에 대비할 수 있습니다.

```bash
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
{% capture capture04 %}
**roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch**

MoveIt이 활성화된 RViz가 실행됩니다. Motion Planning plugin이 실행되어 기존에 moveit_setup_assistant를 통해 미리 저장된 모션 또는 interactive marker을 통해 설정한 모션을 move_group에게 전달할 수 있습니다. 목표 위치를 설정한 후 `Plan and Execute`버튼을 누르면 로봇이 움직이게 됩니다.
{% endcapture %}
<div class="notice--success">{{ capture04 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_rviz.png)

**주의**  
MoveIt의 Interactive Marker를 활용하여 OpenMANIPULATOR-X를 제어하는 경우 MoveIt 소프트웨어의 5DOF 기구학 계산에 한계가있어, 원활한 제어가 가능하지 않을 수 있습니다.
{: .notice--warning}

## ROBOTIS GUI 컨트롤러 실행하기
[Remote PC] RViz를 사용하지 않고 Gazebo와 연결하여 로봇 팔을 제어해보려는 경우 로보티즈 GUI는 OpenMANIPULATOR의 첫번째 DYNAMIXEL을 기준으로 그리퍼의 유효한 파지 위치(그리퍼 사이의 빨간색 육면체)를 레퍼런스로 하는 Task Space Control이나 각 조인트 관절의 각도를 레퍼런스로 하는 Joint Space Control을 지원합니다.
필요에 따라 편리한 제어 방법을 사용할 수 있습니다.

```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
{% capture capture05 %}
**roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch**

유저인터페이스로 c++ move_group_interface를 사용한 qt gui가 실행됩니다. 인터페이스를 통해 받아온 현재 조인트 위치 및 end-effector의 위치가 gui상에 표시됩니다. Send버튼을 클릭하면 설정된 위치값을 인터페이스를 통해 move_group에게 전달하여 컨트롤러에 전달, 로봇이 움직이게 됩니다.
{% endcapture %}
<div class="notice--success">{{ capture05 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/tb3_omx_gui_controller.png)

# 실제 OpenMANIPULATOR-X 제어하기

MoveIt의 move_group이라는 노드는 아래와 같이 여러 정보들을 기반으로 연산된 궤적을 ROS 1에서 지원하는 action 형식으로 로봇 컨트롤러에게 제공하는 적분기(intergrator, 積分器)로서의 역할을 합니다. 사용자는 move_group노드에 moveit이 제공하는 세가지 인터페이스 (C++, Python, RViz Plugin) 를 통해 접근을 할 수 있습니다. 사용자 인터페이스들을 통해 명령을 받으면 move_group노드는 moveit config 정보 (조인트 각도 제한, 기구학 해석, 충돌 감지) 및 로봇의 상태정보를 토대로 궤적을 생성하여 로봇 컨트롤러에 제공합니다.

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/move_group.png)

## roscore 실행하기
[Remote PC] roscore를 실행합니다.
```bash
$ roscore
```

## Bringup 실행하기

[Remote PC] 기본적인 TurtleBot3의 플랫폼과는 달리 OpenMANIPULATOR를 제어할 수 있는 서비스 서버가 필요하기 때문에 아래와 같이 Manipulation을 위한 bringup launch 파일을 실행합니다.

```bash
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
{% capture capture02 %}
**roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch**

**turtlebot3_manipulation_bringup 노드**

turtlebot3_manipulation_bringup.launch를 실행시키면 arm_controller와 gripper_controller 두 개의 컨트롤러가 실행이 됩니다. move_group과 통신하는 action server 컨트롤러의 역할로써 각각 move_group을 통해 팔과 그리퍼 관절의 목표궤적을 읽어들여 순서대로 publish합니다. publish된 토픽들은 OpenCR을 통해 로봇의 관절에 조립된 DYNAMIXEL에 전달되어 OpenMANIPULATOR를 움직이게 됩니다.
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

## move_group 실행하기
[Remote PC] MoveIt과 연동되는 사용자 인터페이스인 move_group 노드를 실행합니다.
```bash
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```
<div class="notice--success">{{ capture03 | markdownify }}</div>

## RViz 실행하기
[Remote PC] 각종 데이터의 시각화와 Interactive Marker를 활용한 OpenMANIPULATOR의 제어를 위해 RViz를 실행합니다.
```bash
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```
<div class="notice--success">{{ capture04 | markdownify }}</div>

## ROBOTIS GUI 실행하기
[Remote PC] RViz와는 별개로 필요한 경우 ROBOTIS GUI를 통해 OpenMANIPULATOR를 제어할 수도 있습니다.
```bash
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
<div class="notice--success">{{ capture05 | markdownify }}</div>
