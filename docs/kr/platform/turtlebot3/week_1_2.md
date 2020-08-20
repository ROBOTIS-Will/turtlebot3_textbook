---
layout: splash
lang: kr
ref: turtlebot3_textbook1-2
permalink: /docs/turtlebot3_textbook/week1-2/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook1-2"
---


# 시뮬레이션에서 터틀봇 구동하기

**주의**  
본 교재는 Ubuntu 16.04와 ROS 1 Kinetic Kame 환경에서 테스트되었습니다.  
또한, 시뮬레이션은 Remote PC에서 실행하시기 바랍니다.
{: .notice--warning}

터틀봇3는 시뮬레이션을 통해 가상의 환경에서 로봇을 프로그래밍하고 개발할 수 있는 환경을 지원합니다. 두 종류의 개발 환경을 선택할 수 있는데, fake 노드와 시각화 도구인 RViz를 사용하는 방법과, 3차원 시뮬레이터인 Gazebo를 사용하는 방법이 있습니다. Fake 노드를 사용하는 경우 로봇 모델과 동작을 테스트하는데 적절하지만, 센서를 사용할 수 없다는 단점이 있습니다. SLAM과 내비게이션을 하는 경우 Gazebo를 권장하는데, 시뮬레이션 내에서 IMU와 LIDAR, 카메라와 같은 각종 센서를 시뮬레이션 할 수 있기 때문입니다.

## Fake Node로 시뮬레이션 하기

turtlebot3_fake_node를 사용하기 위해서는 turtlebot3_simulation 메타 패키지를 설치해야 합니다.  
turtlebot3_simulation 메타 패키지를 사용하기 위해서는 turtlebot3 메타 패키지와 turtlebot3_msgs 패키지가 반드시 설치되어 있어야 합니다.  
[**Remote PC**] 다음의 명령을 사용해서 패키지를 설치하십시오.

```bash
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

[**Remote PC**] 가상의 로봇을 구동하기 위해서는, 아래와 같이 turtlebot3_fake의  turtlebot3_fake.launch 파일을 실행하십시오.  
turtlebot3_fake는 실제 로봇을 사용하지 않고 시뮬레이션할 수 있는 매우 간단한 노드입니다.  
RViz에서 가상의 터틀봇3를 원격제어 노드로 제어할 수 있습니다.

터틀봇3의 모델명을 우선 지정해야 합니다. 아래 명령어 중 ${TB3_MODEL}에 해당하는 부분에 `burger`, `waffle`, `waffle_pi`중 사용하는 모델의 이름을 지정하십시오.

{% capture note06 %}
**참고**  
터틀봇3의 모델명을 파라미터로 저장하려면 `~/.bashrc` 파일에 다음과 같이 추가하십시오.  
아래 예는 burger를 지정하는 경우입니다.  
`.bashrc` 파일에 파라미터가 추가되면 이후 나오는 터틀봇3 모델의 파라미터 설정을 반복할 필요가 없습니다.
```bash
export TURTLEBOT3_MODEL=burger
```
{% endcapture %}
<div class="notice">{{ note06 | markdownify }}</div>

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_fake turtlebot3_fake.launch
```

{% capture note07 %}
**roslaunch turtlebot3_fake turtlebot3_fake.launch**

- publish : odom, joint_states
- subscribe : cmd_vel
{% endcapture %}
<div class="notice--success">{{ note07 | markdownify }}</div>

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

{% capture note08 %}
**roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch**
- publish : cmd_vel

turtlebot3_teleop_key.launch 파일을 실행시켜서 생성된 turtlebot3_teleop_keyboard 노드에서는 키보드의 입력을 읽어와서 linear와 angular 값을 업데이트하고, linear와 angular가 포함된 twist 형식의 topic인 cmd_vel을 publish 합니다.  
turtlebot3_fake 노드에서는 하드웨어의 실제 값이 아닌 계산된 값을 이용해 시뮬레이터가 실제 하드웨어와 유사하게 동작하도록 합니다. 입력된 cmd_vel로 계산된 가상의 Turtlebot3의 odom과 joint_states 값을 publish합니다.
{% endcapture %}
<div class="notice--success">{{ note08 | markdownify }}</div>

### Gazebo에서 터틀봇3 시뮬레이션하기

Gazebo를 사용한 시뮬레이션은 두 가지 방법에 있는데, ROS에서 turtlebot3_gazebo 패키지를 사용하는 방법과 ROS를 사용하지 않고 turtlebot3_gazebo_plugin을 사용하는 방법이 있습니다.  
여기에서는 ROS와 turtlebot3_gazebo를 사용하는 방법을 설명합니다.

#### Gazebo용 ROS 패키지
Remote PC에서 처음으로 Gazebo를 실행하는 경우 프로그램이 시작하는데 오래 걸릴 수 있습니다.

##### A. Empty World 예제
[**Remote PC**] 아래의 명령어로 터틀봇3를 Gazebo 상의 빈 공간에 불러올 수 있습니다.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_empty_world.png)

{% capture note09 %}
**roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch**
- publish : joint_states, odom, scan, tf
- subscribe : cmd_vel

turtlebot3_empty_world.launch를 실행하면 파일 내부의 설정에 따라 Gazebo 시뮬레이터가 실행되며, 설정된 TurtleBot3 모델이 Gazebo 시뮬레이터에 생성됩니다.  
이때 사용되는 XACRO(XML Macro) 형식의 URDF 파일의 설정에 따라 imu, scan, odom, joint_states, tf의 topic을 publish하고 cmd_vel를 subscribe하게 됩니다.  
이후 teleop 노드를 통하여 cmd_vel를 publish하면 Gazebo 시뮬레이터가 그 값을 subscribe하게 되고 시뮬레이터에 생성된 로봇이 움직이게 되는 것을 볼 수 있습니다.
{% endcapture %}
<div class="notice--success">{{ note09 | markdownify }}</div>

##### B. TurtleBot3 World 예제
[**Remote PC**] 아래의 명령어로 터틀봇3를 터틀봇 심볼형태의 공간에 불러올 수 있습니다.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_bugger.png)

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_world_waffle.png)

##### C. TurtleBot3 House 예제
[**Remote PC**] 아래의 명령어로 터틀봇3를 가상의 건물에 불러올 수 있습니다.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house.png)

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_house1.png)

#### 터틀봇3 구동하기
[**Remote PC**] 새 터미널 창을 열어서 아래 명령어로 turtlebot3_teleop_key 노드를 실행합니다.

```bash
$ export TURTLEBOT3_MODEL=%{TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

#### 충돌 회피
[**Remote PC**] TurtleBot3 World에서 터틀봇을 자율 이동시키려면 실행중인 모든 터미널 창을 닫고 새로운 터미널 창에서 다음 명령을 입력하십시오.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

[**Remote PC**] 새로운 터미널 창을 열고 아래의 충돌 회피 시뮬레이션 노드를 실행시키세요.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```

#### RViz 실행하기
[**Remote PC**] RViz는 시뮬레이션이 실행되는 동안 publish된 topic의 데이터를 시각화합니다.  
새로운 터미널 창에서 아래의 명령어를 입력하면 RViz를 실행할 수 있습니다.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)

## 실제 로봇과 시뮬레이션 교차 개발

### 목표
- 20대의 터틀봇과 40대의 simulation 사이에 원활한 교대 개발이 가능한 환경을 제시 및 네트워크 구성안 제시
- 교차 개발을 위한 실행방법 제시

#### 네트워크 구성안

##### 가정
- 모든 TB3와 사용자PC(Simulation)은 고정 IP를 가진다.
- 사용자 PC는 고정IP가 아니어도 동작 가능할 것으로 보이나 ROS 네트워크 세팅을 편하게 하기 위해 고정으로 사용하는 것을 추천
- 각 사용자는 자신이 사용해야 할 TB3의 IP 주소를 알고 있다.

1. 1안
공유기를 여러대 사용(5대), 4대의 TB3와 8명의 사람(Simulator 포함)가 하나의 공유기 사용 (추천)

2. 2안
고성능 공유기에 40명의 사람과 20대의 TB3 모두 연결(고객 요구)


#### 네트워크 세팅(1안)
1. 공유기 세팅
- SSID 및 공유기 IP (IP 대역) : 공유기마다 설정방법이 달라 자세한 설명은 생략
  - Turtlebot_Server_1 : 10.17.1.1 (10.17.1.x)
  - Turtlebot_Server_2 : 10.17.2.1 (10.17.2.x)
  - Turtlebot_Server_3 : 10.17.3.1 (10.17.3.x)
  - Turtlebot_Server_4 : 10.17.4.1 (10.17.4.x)
  - Turtlebot_Server_5 : 10.17.5.1 (10.17.5.x)

2. Turtlebot3 네트워크 세팅
- IP 주소 예시 (Turtlebot_Server_1의 경우)
    - [Turtlebot3]의 IP : 4대
      - 10.17.1.11  
      - 10.17.1.12  
      - 10.17.1.13  
      - 10.17.1.14  

    - [User PC]의 IP : 8대
      - 10.17.1.21  
      - 10.17.1.22  
      - 10.17.1.23  
      - 10.17.1.24  
      - 10.17.1.25  
      - 10.17.1.26  
      - 10.17.1.27  
      - 10.17.1.28  

    - 위와 같은 방식의 주소로 나머지 4대의 서버에 연결된 Turtlebot3와 사용자 PC의 IP를 세팅

- 네트워크 세팅
  - [Turtlebot3] (10.17.1.11의 경우)
      1. Turtlebot에 키보드, 마우스, 모니터를 연결 한 후 라즈베리파이를 부팅
      2. 터미널 창을 열어 `/etc/dhcpcd.conf` 파일의 `static IP` 부분을 변경
```bash
$ sudo nano /etc/dhcpcd.conf
```
      3. 아래 내용을 추가
```bash
interface wlan0
static ip_address=10.17.1.11/24
static routers=10.17.1.1
static domain_name_servers=8.8.8.8
```
      4. 파일 저장 후 재부팅
```bash
$ sudo reboot
```
      5. 네트워크 매니저에서 아래와 같이 설정  
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/002.png)
      6. 공유기에 연결 후 IP 주소 확인
```bash
$ ifconfig
```
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/003.png)

#### 네트워크 세팅(2안)
1. 공유기 세팅
- SSID 및 공유기 IP (IP 대역) : 공유기마다 설정방법이 달라 자세한 설명은 생략
  - Turtlebot_Server : 10.17.1.1 (10.17.1.x)

2 Turtlebot3 네트워크 세팅 (편의성을 위해 고정 IP 사용)
- IP 주소 예시 
  - [Turtlebot3]의 IP : 20대
    - 10.17.1.11 ~ 30
  - [User PC]의 IP : 40대
    - 10.17.1.101 ~ 140
- 네트워크 세팅
  - [Turtlebot3] (10.17.1.11의 경우)
    1. Turtlebot에 키보드, 마우스, 모니터를 연결 한 후 라즈베리파이를 부팅
    2. 터미널 창을 열어 `/etc/dhcpcd.conf` 파일의 `static IP` 부분을 변경
```bash
$ sudo nano /etc/dhcpcd.conf
```
    3. 아래 내용을 추가
```bash
interface wlan0
static ip_address=10.17.1.11/24
static routers=10.17.1.1
static domain_name_servers=8.8.8.8
```
    4. 파일 저장 후 재부팅
```bash
$ sudo reboot
```
    5. 공유기에 연결 및 IP 주소 확인. 터미널을 열어 아래와 같이 입력
```bash
$ ifconfig
```
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/004.png)

  - [User PC] (10.17.1.101의 경우)
    1. 네트워크 매니저에서 아래와 같이 설정 후 저장  
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/005.png)
    2. 공유기에 연결 후 IP 주소 확인
```bash
$ ifconfig
```
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/006.png)

### ROS 네트워크 세팅

여기에서 ROS 네트워크 설정은 이전에 사용한 방법과는 다른 새로운 방법입니다.  
이 방법을 적용하면 앞으로 RemotePC에서 터틀봇 SBC에 연결한 후 `roslaunch turtlebot3_bringup turtlebot3_robot.launch`명령을 수행 할 필요가 없습니다.  
설정이 완료되면 Remote PC에서 직접 아래의 명령을 TurtleBot3의 IP 주소와 함께 입력하여 네트워크에 연결된 TurtleBot3을 구동 할 수 있습니다.

```bash
roslaunch turtlebot3_bringup turtlebot3_robot_machine.launch address:=${IP_ADDRESS_of_TB3}
```

1. [Turtlebot3]
- 추가적으로 ROS 네트워크 세팅을 할 필요는 없음

2. [User PC]
- 참고 : [eManual](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#network-configuration)
- 사용자 PC의 IP 예시 : 10.17.1.101
- 사용자 PC의 IP 확인(아래 명령어 입력시 위에서 세팅한 사용자 PC의 정적 IP가 출력될 것임)
```bash
$ ifconfig
```
- `~/.bashrc` 파일 수정 : 아래의 문구가 있다면 수정하고 없으면 추가  
```bash
export ROS_MASTER_URI=http://10.17.1.101:11311
export ROS_HOSTNAME=10.17.1.101
```

### 기타 설정
1. [Turtlebot3]
  - machine 태그를 이용한 launch 파일에서 사용하는 `env.bash` 파일 생성(생성 위치 : /home/pi)  

    ```bash
    $ nano ~/env.bash
    ```
    ```
    #!/bin/bash

    # check if other turtlebot3_core is already running
    is_running=`ps ax | grep turtlebot3_core`
    IFS=' ' read -ra is_runnings <<< "$is_running"
    process_name=${is_runnings[4]}
    if [ ${process_name} == "python" ]
    then
      echo "other turtlebot3_core is already running."
      exit 1
    fi

    #### ROS ####
    source /opt/ros/kinetic/setup.bash
    source ~/catkin_ws/devel/setup.bash

    #### ROS Network ####
    ip_address=`hostname -I`
    ip_address_trim=${ip_address%% * }
    ip_address_no_space="$(echo -e "${ip_address_trim}" | tr -d '[:space:]')"
    export ROS_HOSTNAME=${ip_address_no_space}

    ##### Set TURTLEBOT3 Model ####
    export TURTLEBOT3_MODEL=waffle_pi

    exec "$@"
    ```
- `env.bash` 파일에 실행권한 추가  
```bash
$ chmod +x ~/env.bash
```

2. [User PC]
- 연결할 Turtlebot3에 대한 ssh key 생성, 아래에서 만들 script를 이용 
- keygen script 제작 : ssh-keyscan으로 여러 종류의 알고리즘을 이용한 key 생성

    - script 생성

    ```bash
    $ nano ~/tb3_ssh_keygen
    ```

    ```
    #!/bin/bash
    argc=$#
    args=("$@")

    if [ 0 -eq $argc ]
    then   	 
      echo "need to argument that host ip for ssh connection"
      echo "Usage: $0 [ip address] ..."
      exit 1
    fi

    for((index = 0; index < $#; index++ ))
    do
      ssh-keygen -R ${args[$index]}
      ssh-keyscan ${args[$index]} >> ~/.ssh/known_hosts
    done
    ``` 

    - 실행권한 추가  
    ```bash
    $ chmod +x ~/tb3_ssh_keygen
    ```

    - script 실행 (ex. TB3의 IP가 10.17.3.11~30인 경우, 20대의 TB3에 고정 IP 세팅 후 TB3가 켜진 상태에서 명령어 실행해야 함)  
    ```bash
    $ ~/tb3_ssh_keygen 10.17.3.11 10.17.3.12 10.17.3.13 10.17.3.14 10.17.3.15\
    10.17.3.16 10.17.3.17 10.17.3.18 10.17.3.19 10.17.3.20\
    10.17.3.21 10.17.3.22 10.17.3.23 10.17.3.24 10.17.3.25\
    10.17.3.26 10.17.3.27 10.17.3.28 10.17.3.29 10.17.3.30
    ```

- turtlebot3_robot_machine.launch 제작
  1. machine 태그는 node용 태그여서 include 태그에서는 작동하지 않음, turtlebot3_robot.lauch 파일 수정 필요
  2. [Turtlebot3] turtlebot3_robot.launch 대체 용도, 실행시 Turtlebot에 직접 연결할 필요가 없고 Turtlebot3의 ROS 네트워크 세팅을 변경할 필요가 없다.
  3. turtlebot3/turtlebot3_bringup/launch 폴더에 아래의 파일을 생성

```bash
$ nano turtlebot3_robot_machine.launch
```

```bash
<?xml version="1.0"?>
<launch>
  <arg name="multi_robot_name" default=""/>
  <arg name="set_lidar_frame_id" default="base_scan"/>
  <arg name="address" default="10.17.3.91"/>
  <arg name="env_name" default="env.bash"/>
  <arg name="user_name" default="pi"/>
  <arg name="password" default="turtlebot"/>
 
  <!-- setting for machine -->
  <machine name="tb3" address="$(arg address)" env-loader="~/$(arg env_name)" user="$(arg user_name)" password="$(arg password)" />

  <!-- packages for turtlebot3 -->
  <node machine="tb3" pkg="rosserial_python" type="serial_node.py" name="turtlebot3_core" output="screen">
	<param name="port" value="/dev/ttyACM0"/>
	<param name="baud" value="115200"/>
	<param name="tf_prefix" value="$(arg multi_robot_name)"/>
  </node>
 
  <node machine="tb3" pkg="hls_lfcd_lds_driver" type="hlds_laser_publisher" name="turtlebot3_lds" output="screen">
	<param name="port" value="/dev/ttyUSB0"/>
	<param name="frame_id" value="$(arg set_lidar_frame_id)"/>
  </node>

  <node machine="tb3" pkg="turtlebot3_bringup" type="turtlebot3_diagnostics" name="turtlebot3_diagnostics" output="screen"/>

</launch>
```

### Turtlebot3, Simulation 전환 실행 방법
모든 실행은 [Remote PC]에서 진행함

1. TB3 실행법

    - roscore
    터미널을 열어 아래 명령어 입력
        ```
        $ roscore
        ```

    - 터틀봇 원격 구동 : turtlebot3_robot_machine.launch.  
    다른 터미널을 열어 아래 명령어 입력(연결 하려는 turtlebot3의 IP 주소가 10.17.1.11로 가정)  
    ```
    $ roslaunch turtlebot3_bringup turtlebot3_robot_machine.launch address:=10.17.1.11
    ```

    ![](http://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)

    1. SSH 에러 발생 시 다음 명령어 입력 후 다시 실행
        ```
        $ ~/tb3_ssh_keygen 10.17.3.11
        ```
    2. 다른 사용자가 Turtlebot을 사용 중인 경우 launch 파일 실행 시 아래의 메시지가 나오며 종료
        ```
        RLException: remote roslaunch failed to launch: tb3
        The traceback for the exception was written to the log file
        ```

    - Robot Model & TF : turtlebot3_remote.launch.  
    다른 터미널을 열어 아래의 명령어 입력
        ```
        $ roslaunch turtlebot3_bringup turtlebot3_remote.launch
        ```

    - RVIZ  
    새로 터미널을 열어 아래 명령어 입력(Waffle Pi를 사용하는 경우 ${TB3_MODEL} 대신 waffle_pi 입력)
        ```
        $ export TURTLEBOT3_MODEL=${TB3_MODEL}
        $ rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
        ```

2. Simulation(Gazebo) 실행법
참고 : [eManual](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo)

    - roscore  
    터미널을 열어 아래 명령어 입력
        ```
        $ roscore
        ```

    - Gazebo 실행 
    다른 터미널을 열어 아래 명령어 입력(Waffle Pi를 사용하는 경우 ${TB3_MODEL} 대신 waffle_pi 입력)
        ```
        $ export TURTLEBOT3_MODEL=${TB3_MODEL}
        $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
        ```

    - Rviz  
    새로 터미널을 열어 아래 명령어 입력(Waffle Pi를 사용하는 경우 ${TB3_MODEL} 대신 waffle_pi 입력)
        ```
        $ export TURTLEBOT3_MODEL=${TB3_MODEL}
        $ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
        ```
    ![](http://emanual.robotis.com/assets/images/platform/turtlebot3/simulation/turtlebot3_gazebo_rviz.png)


3. 사용 전환
    - 실시간 전환 : gazebo는 /use_sim_time 파라미터를 사용해 실시간 전환(roscore를 유지한 사용 전환)이 어려움
    - 전환 방법
        - roscore를 포함한 모든 node를 종료(실행한 터미널에서 `CTRL`+`C`키 입력)
        - 위에 설명한 실행법을 따라 전환
    - TurtleBot3 실로봇과 Simulation 비교

|      | TurtleBot3  | 시뮬레이션(Gazebo) |
|:----:|:----------------|:------------------------|
| 환경 | 다양한 실제 환경 |Gazebo환경<br />- 제공<br />&nbsp;&nbsp;- Empty World<br />&nbsp;&nbsp;- Turtlebot3 World<br />&nbsp;&nbsp;- Turtlebot3 House<br />- 유저가 제작한 환경<br />![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/008.png)|
|모델|Burger<br/>Waffle<br/>Waffle Pi|Burger<br/>Waffle<br/>Waffle Pi|
|센서 및 토픽 이름|LIDAR : /scan<br />IMU : /imu<br />CAMERA(Waffle Pi) : /raspicam_node/image/compressed|LIDAR : /scan<br />IMU : /imu<br />CAMERA(Waffle, Waffle Pi) : <br />&nbsp;&nbsp;/camera/rgb/image_raw,<br />&nbsp;&nbsp;/camera/rgb/image_raw/compressed
|사용기기|Turtlebot3(Burger, Waffle, Waffle Pi)<br />Remote PC(User PC)|Remote PC(User PC)|