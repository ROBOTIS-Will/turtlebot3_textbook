---
layout: splash
lang: kr
ref: turtlebot3_textbook1-1
permalink: /docs/turtlebot3_textbook/week1-1/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook1-1"
---

# ROS 개발환경 설치(Remote PC)

**참고자료**: http://wiki.ros.org/kinetic/Installation/Ubuntu
{: .notice}

## ROS 설치와 환경설정

ROS Kinetic 버전은 Ubuntu 15.10, Ubuntu 16.04, Debian 8을 기반으로 동작하나, Ubuntu 16.04를 사용하는 것을 추천합니다.

### sources.list 설정하기
아래 커맨드를 입력하면 설치하려는 ROS의 최신 버전 패키지를 얻어올 수 있습니다.

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 키(Key) 설정하기
Ubuntu 서버에 접속하기 위한 키를 설정합니다.

```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### ROS 설치하기
우선 Ubuntu의 패키지 인덱스를 최신으로 업데이트합니다.

```bash
$ sudo apt-get update
```

필요한 ROS 패키지를 하나씩 따로 설치할 수 있지만, 일반적인 경우 아래와 같이 desktop-full 패키지를 설치하는 것을 권장합니다.

```bash
$ sudo apt-get install ros-kinetic-desktop-full
```

### 패키지 빌드에 필요한 Dependency 설치하기
추가적으로 아래 패키지를 설치하면 ROS의 작업공간인 workspace를 생성하고 관리하거나 ROS 패키지와 관련된 다른 패키지들을 다운로드하는데 도움이 됩니다.

```bash
$ sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### rosdep 초기화 하기
ROS를 사용하기 이전에 rosdep을 초기화해야 합니다. rosdep은 ROS로 작성된 코드를 컴파일하거나 실행할 때 필요한 관련 패키지(공식적으로 dependency 라고 칭합니다)의 설치를 도와줍니다.

```bash
$ sudo rosdep init
$ rosdep update
```

### ROS 환경 설정
아래와 같이 ROS에 대한 설정이 저장된 환경파일이 shell을 실행할 때마다 자동으로 실행되도록 .bashrc 파일에 추가합니다.

```bash
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

아래는 현재 열린 shell에서 .bashrc 파일을 다시 읽어들이는 명령어입니다.

```bash
$ source ~/.bashrc
```

## 기본적인 ROS 명령어

**참고자료**: https://w3.cs.jmu.edu/spragunr/CS354_S19/handouts/ROSCheatsheet.pdf
{: .notice}

|:----|:----|
|**명령어**|**설명**|
|roscore|단일 명령어로 사용되며 ROS Master와 ROS 실행에 필요한 각종 서버를 실행시킵니다.|
|rosrun|명시된 node를 실행시킵니다.|
|roslaunch|파일에 명시된 여러 개의 node를 옵션과 함께 실행시킵니다.|
|rosclean|ROS의 log 파일을 확인하거나 삭제합니다.|
|roscd|명시된 ROS 패키지가 저장된 디렉토리로 이동합니다.|
|rostopic|ROS의 topic 정보를 확인합니다.|
|rosservice|ROS의 service 정보를 확인합니다.|
|rosnode|ROS의 node 정보를 확인합니다.|
|rosparam|ROS의 파라미터(parameter) 정보를 확인합니다.|
|rosbag|ROS 내부의 메세지를 기록하거나 재생합니다.|
|rosmsg|ROS의 message 데이터 구조를 보여줍니다.|
|rossrv|ROS의 service 데이터 구조를 보여줍니다.|
|catkin_create_pkg|자동으로 ROS 패키지와 관련 파일을 생성합니다.|
|catkin_make|catkin 빌드 시스템으로 패키지를 빌드합니다.|
|rospack|명시된 ROS 패키지의 정보를 확인합니다.|

# 터틀봇3 ROS 설치
라즈베리 파이 3B+와 라즈비안 OS의 조합으로 ROS1 Kinetic 설치를 진행합니다.  
이미지 파일에는 ROS와 터틀봇3 필수 패키지들이 포함되어 있습니다.  
다른 SBC와 OS를 사용하는 경우 각 소프트웨어와 하드웨어의 매뉴얼을 참고하시기 바랍니다.  

1. [터틀봇3 라즈비안 이미지 다운로드](http://www.robotis.com/service/download.php?no=1738)
2. 다운로드 완료 후 압축파일 해제
3. [Etcher](https://etcher.io/) 또는 Win32 DIsk Imager([Linux](https://elinux.org/RPi_Easy_SD_Card_Setup#Using_the_Linux_command_line), [Windows](https://elinux.org/RPi_Easy_SD_Card_Setup#Using_the_Win32DiskImager_program)) 툴을 이용해서 이미지 파일을 SD카드에 복사
4. [공식 가이드](https://projects.raspberrypi.org/en/projects/raspberry-pi-setting-up/4)에 따라 설정 진행
5. 여기서부터는 라즈베리 파이와 연결된 모니터와 입력장치를 활용해서 라즈베리 파이의 터미널에서 실행합니다.
6. 아래 환경설정 명령어 입력 후 `7 Advanced Options` > `A1 Expand Filesystem` 선택
```bash
$ sudo raspi-config
```
7. Network Time Protocol(NTP) 서버 설치 및 동기화
```bash
$ sudo apt-get install ntpdate
$ sudo ntpdate ntp.ubuntu.com
```
8. 설정이 완료되면 라즈베리 파이에 연결된 모니터와 입력장치를 제거하셔도 좋습니다.

# 네트워크 설정

## PC 네트워크 설정

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration.png)

TurtleBot3의 경우 ROS Master가 원격 PC에서 구동됩니다.  
ROS기반의 TurtleBot3와 Remote PC가 서로 통신하기 위해서는 IP 주소가 설정되어 있어야 합니다.  
이때, Remote PC와 TurtleBot3의 PC(또는 SBC)는 동일한 라우터에 연결되어 같은 무선 네트워크에 접속되어야 합니다.  
네트워크에 연결되면 아래 명령어를 Remote PC의 터미널 창에 입력해서 IP 주소를 찾습니다.

```bash
$ ifconfig
```

아래 표시된 부분의 IP 주소가 Remote PC의 IP 주소입니다.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration2.png)

환경설정 파일의 네트워크 설정 부분을 수정합니다.
nano 에디터가 설치되어있지 않은 경우 아래 명령어로 에디터를 먼저 설치합니다.
```bash
$ sudo apt install nano
```

```bash
$ nano ~/.bashrc
```

`ALT` + `/` 단축키를 누르면 파일의 마지막 부분으로 이동합니다.  
아래와 같이 Remote PC의 IP주소를 `ROS_MASTER_URI`와 `ROS_HOSTNAME` 항목에 입력합니다.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration3.png)

`CTRL` + `X` 단축키로 수정을 끝내면 저장여부를 묻는 화면에서 `Y`를 누르고 `Enter`를 눌러 동일한 파일에 덮어쓴 후 종료합니다.

마지막으로 아래 명령을 사용하여 bashrc의 변경사항을 현재의 터미널 창에 적용합니다.

```bash
$ source ~/.bashrc
```

## 터틀봇3 SBC 네트워크 설정

Remote PC에서 터틀봇3 SBC의 터미널에 접근하려면 아래와 같이 ssh 명령과 터틀봇3 SBC의 IP주소를 입력하면 됩니다.  
기본 설정된 비밀번호는 소문자 `turtlebot` 입니다.
```bash
ssh pi@192.168.0.200
```

아래 명령어를 터미널 창에 입력해서 라즈베리 파이의 환경설정 파일을 수정합니다.

```bash
$ nano ~/.bashrc
```

`ALT` + `/` 단축키를 누르면 파일의 마지막 부분으로 이동합니다.  
아래와 같이 Remote PC의 IP주소를 `ROS_MASTER_URI`에 입력하고, 라즈베리 파이의 IP주소를 `ROS_HOSTNAME` 항목에 입력합니다.

![](https://emanual.robotis.com/assets/images/platform/turtlebot3/software/network_configuration5.png)

`CTRL` + `X` 단축키로 수정을 끝내면 저장여부를 묻는 화면에서 `Y`를 누르고 `Enter`를 눌러 동일한 파일에 덮어쓴 후 종료합니다.

마지막으로 아래 명령을 사용하여 bashrc의 변경사항을 현재의 터미널 창에 적용합니다.

```bash
$ source ~/.bashrc
```

# 터틀봇3 구동하기

터틀봇3를 구동하기에 앞서 터틀봇3에 필요한 ROS 패키지를 설치해야 합니다.
```bash
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino \
ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base \
ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport \
ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation \
ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

## Bringup

아래 명령어를 실행할 때 Remote PC와 TurtleBot3 SBC를 잘 구분하시기 바랍니다. 특히, roscore는 Turtlebot3 SBC에서 실행하지 마십시오.  
명령어를 실행하기에 앞서 각 기기(Turtlebot3 SBC, Remote PC)의 IP 주소가 올바르게 설정되어 있는지 확인하십시오.  
배터리의 전압이 11V보다 낮으면 경고음이 지속적으로 발생합니다. 저전압 경고음이 울리면 사용을 중단하고 배터리를 충전해야 합니다.

### roscore 실행하기

[**Remote PC**] 아래 명령어로 `roscore`를 실행합니다.

```bash
$ roscore
```

### 터틀봇3 Bringup

[**터틀봇3 SBC**] 다음의 설치 방법은 ROS 1 Kinetic에서만 사용할 수 있습니다.

{% capture note01 %}
- Remote PC에서 라즈비안 이미지를 사용하여 TurtleBot3를 구동하는 경우 다음 명령을 TurtleBot3의 SBC에서 실행하십시오.  
명령을 실행하면 TurtleBot3 패키지의 `kinetic-devel` 브랜치 내용으로 업데이트됩니다. 이 작업을 수행하기 위해서는 인터넷에 연결되어 있어야합니다.  
  ```bash
  $ cd ~/catkin_ws/src && rm -rf turtlebot3
  $ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
  $ cd ~/catkin_ws/src/turtlebot3
  $ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
  $ cd ~/catkin_ws && catkin_make -j1
  $ source ~/.bashrc
  ```

- git이 설치되어 있지 않아 오류가 발생하는 경우 다음 명령으로 git을 설치하십시오.  
  ```bash
  $ sudo apt install git
  ```
{% endcapture %}
<div class="notice--warning">{{ note01 | markdownify }}</div>

터틀봇3 모델이 Burger인 경우, bringup이 정상적으로 실행되면 터미널에서 다음과 같은 메세지가 표시됩니다.

```bash
SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.13
 * /turtlebot3_core/baud: 115200
 * /turtlebot3_core/port: /dev/ttyACM0
 * /turtlebot3_core/tf_prefix: 
 * /turtlebot3_lds/frame_id: base_scan
 * /turtlebot3_lds/port: /dev/ttyUSB0

NODES
  /
    turtlebot3_core (rosserial_python/serial_node.py)
    turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
    turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS_MASTER_URI=http://192.168.1.2:11311

process[turtlebot3_core-1]: started with pid [14198]
process[turtlebot3_lds-2]: started with pid [14199]
process[turtlebot3_diagnostics-3]: started with pid [14200]
[INFO] [1531306690.947198]: ROS Serial Python Node
[INFO] [1531306691.000143]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1531306693.522019]: Note: publish buffer size is 1024 bytes
[INFO] [1531306693.525615]: Setup publisher on sensor_state [turtlebot3_msgs/SensorState]
[INFO] [1531306693.544159]: Setup publisher on version_info [turtlebot3_msgs/VersionInfo]
[INFO] [1531306693.620722]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1531306693.642319]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1531306693.687786]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1531306693.706260]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1531306693.722754]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1531306693.759059]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1531306695.979057]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1531306696.007135]: Note: subscribe buffer size is 1024 bytes
[INFO] [1531306696.009083]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1531306696.040047]: Setup subscriber on sound [turtlebot3_msgs/Sound]
[INFO] [1531306696.069571]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1531306696.096364]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1531306696.390979]: Setup TF on Odometry [odom]
[INFO] [1531306696.394314]: Setup TF on IMU [imu_link]
[INFO] [1531306696.397498]: Setup TF on MagneticField [mag_link]
[INFO] [1531306696.400537]: Setup TF on JointState [base_link]
[INFO] [1531306696.407813]: --------------------------
[INFO] [1531306696.411412]: Connected to OpenCR board!
[INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
[INFO] [1531306696.418398]: --------------------------
[INFO] [1531306696.421749]: Start Calibration of Gyro
[INFO] [1531306698.953226]: Calibration End
```

{% capture note02 %}
**roslaunch turtlebot3_bringup turtlebot3_robot.launch**

1. **turtlebot3_core.launch**
  - subscribe : cmd_vel
  - publish : joint_states, odom
2. **turtlebot3_lidar.launch**
  - publish : scan

turtlebot3_robot.launch 파일을 실행시키면 turtlebot3_core.launch와 turtlebot3_lidar.launch 파일이 실행되며, TurtleBot3의 상태를 체크하는 노드인 turtlebot3_diagnostics가 생성되어 TurtleBot3의 각종 센서 및 하드웨어에 대한 상태정보를 송신(publish) 합니다.  turtlebot3_core.launch 파일에서는 OpenCR과 통신하여 joint_states, odom을 송신(publish)하고 cmd_vel를 수신(subscribe)하는 노드가 생성됩니다. turtlebot3_lidar.launch 파일에서는 LIDAR를 작동시켜 센서로부터 얻어진 scan데이터를 송신(publish)하는 노드가 생성됩니다.
{% endcapture %}
<div class="notice--success">{{ note02 | markdownify }}</div>

메세지에서 LIDAR(scan) 관련 센서 오류가 발생하는 경우 Bringup 과정을 반복하거나 터틀봇3의 LDS-01 센서가 잘 연결되어 있는지 확인 후 OpenCR을 리셋하시기 바랍니다.

[**Remote PC**] 아래의 명령어는 Bringup이 정상적으로 실행되었을 때 터틀봇과 Remote PC간에 사용되는 Topic 목록을 나타냅니다.

```bash
$ rostopic list -v

Published topics:
 * /rpms [std_msgs/UInt16] 1 publisher
 * /version_info [turtlebot3_msgs/VersionInfo] 1 publisher
 * /battery_state [sensor_msgs/BatteryState] 1 publisher
 * /joint_states [sensor_msgs/JointState] 1 publisher
 * /rosout [rosgraph_msgs/Log] 3 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /cmd_vel_rc100 [geometry_msgs/Twist] 1 publisher
 * /firmware_version [turtlebot3_msgs/VersionInfo] 1 publisher
 * /imu [sensor_msgs/Imu] 1 publisher
 * /odom [nav_msgs/Odometry] 1 publisher
 * /scan [sensor_msgs/LaserScan] 1 publisher
 * /diagnostics [diagnostic_msgs/DiagnosticArray] 2 publishers
 * /tf [tf/tfMessage] 1 publisher
 * /sensor_state [turtlebot3_msgs/SensorState] 1 publisher
 * /magnetic_field [sensor_msgs/MagneticField] 1 publisher

Subscribed topics:
 * /firmware_version [turtlebot3_msgs/VersionInfo] 1 subscriber
 * /motor_power [std_msgs/Bool] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /sound [turtlebot3_msgs/Sound] 1 subscriber
 * /reset [std_msgs/Empty] 1 subscriber
 * /imu [sensor_msgs/Imu] 1 subscriber
 * /scan [sensor_msgs/LaserScan] 1 subscriber
 * /cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /sensor_state [turtlebot3_msgs/SensorState] 1 subscriber
```

[**Remote PC**] 아래 명령어를 사용하면 현재 실행되고 있는 각종 노드와 메세지에 대한 정보를 다이어그램으로 볼 수 있습니다.

```bash
$ rqt_graph
```

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/009.png)

### RViz에서 터틀봇3 실행하기

[**Remote PC**] Bringup을 실행하기에 앞서 터틀봇3의 모델명을 지정해야 합니다.  
아래 명령어 중 ${TB3_MODEL}에 해당하는 부분에 `burger`, `waffle`, `waffle_pi`중 사용하는 모델의 이름을 지정하십시오.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
```

{% capture note03 %}
**roslaunch turtlebot3_bringup turtlebot3_remote.launch**
1. **turtlebot3_remote.launch**
  - **urdf** : Unified Robot Description Format의 약자로써 로봇의 구성과 연결형태를 표현하는 XML 형식의 파일입니다.
  - **robot_state_publisher** : robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf(transform)형식으로 publish 합니다.
  - **subscribe** : joint_states 
  - **publish** : tf

turtlebot3_remote.launch 파일을 실행시키면 로봇의 urdf를 정의된 위치에서 불러옵니다. 또한, joint_states와 urdf를 이용하여 tf를 publish하는 robot_state_publisher 노드를 생성합니다.
{% endcapture %}
<div class="notice--success">{{ note03 | markdownify }}</div>

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/010.png)

[**Remote PC**] 새로운 터미널 창을 열어서 다음 명령어를 입력하십시오.  
아래 명령어를 사용하면 RViz의 설정파일을 불러와서 터틀봇을 RViz로 시각화 할 수 있습니다.  

**참고**
터미널 프로그램은 Ubuntu 왼쪽 상단 시작버튼의 Ubuntu 검색 아이콘에서 찾을 수 있습니다. 터미널을 실행하는 단축키는 `Ctrl` - `Alt` - `T`입니다.
{: .notice}

```bash
$ rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz
```

{% capture note04 %}
**rosrun rviz rviz -d \`rospack find turtlebot3_description\`/rviz/model.rviz**

- subscribe : tf, scan

rviz를 실행시키면 tf와 scan데이터를 각각 로봇의 자세와 주변의 장애물에 대한 정보로 시각화합니다.
{% endcapture %}
<div class="notice--success">{{ note04 | markdownify }}</div>

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/bringup/run_rviz.jpg)

## 키보드를 활용한 원격제어

터틀봇3는 다양한 장치로 원격 제어를 할 수 있습니다. 터틀봇3에 기본으로 제공되는 하드웨어인 DYNAMIXEL, Raspberry Pi 3 B+, OpenCR1.0, Ubuntu Mate16.04 (ROS Kinetic) 환경에서 PS3, XBOX 360, ROBOTIS RC100 등의 컨트롤러에서 테스트되었습니다.

[**Remote PC**] 아래 명령어를 사용해서 turtlebot3_teleop_key 노드를 실행합니다.

```bash
$ export TURTLEBOT3_MODEL=%{TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

{% capture note05 %}
**roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch**
- publish : cmd_vel

turtlebot3_teleop_key.launch 파일을 실행시켜서 생성된 turtlebot3_teleop_keyboard 노드에서는 키보드의 입력을 읽어와서 linear와 angular 값을 업데이트하고, linear와 angular가 포함된 twist 형식의 topic인 cmd_vel을 publish 합니다.  
이후 터틀봇의 SBC에서 실행된 turtlebot3_robot.launch에 포함된 turtlebot3_core.launch에서 cmd_vel을 수신받습니다.  
cmd_vel 토픽은 rosserial을 통해 OpenCR로 전달되고, OpenCR에 업로드된 펌웨어에서 DYNAMIXEL을 제어하기 위한 명령으로 출력됩니다.  
수신받은 명령에 따라 바퀴와 연결된 DYNAMIXEL이 구동하며 로봇을 움직입니다.
{% endcapture %}
<div class="notice--success">{{ note05 | markdownify }}</div>

노드가 성공적으로 실행되면 아래와 같은 메세지가 나타나며, 키보드로 터틀봇3를 제어할 수 있게 됩니다.

```bash
Control Your Turtlebot3!
---------------------------
Moving around:
        w
    a   s   d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```
