---
layout: splash
lang: en
ref: turtlebot3_textbook2-1
permalink: /docs/turtlebot3_textbook/week2-1/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook2-1"
---

# [Class 3] LRF(LDS)센서

TurtleBot3에 사용된 LDS(LASER Distance Sensor)는 2차원 평면의 360도 거리값을 읽을 수 있는 센서로써 SLAM과 Navigation에 필요한 중요한 정보를 제공합니다. LDS 센서는 USB 인터페이스인 USB2LDS 보드를 통해 Raspberry Pi와 USB 케이블을 통해 연결되지만, UART 인터페이스를 통해 OpenCR과도 직접 연결될 수 있습니다.

센서의 특성상 직사광선이 강하게 내리쬐는 외부의 환경에서는 사용이 어렵기 때문에 10,000lux 이하 밝기의 실내공간에서 주행하는 로봇에 적합합니다.

TurtleBot3의 LDS센서는 패키지의 형태로 설치하거나 소스 코드를 다운로드 받은 후 빌드할 수 있습니다.

## 센서 패키지 설치
```bash
$ sudo apt-get install ros-kinetic-hls-lfcd-lds-driver
```
TurtleBot3의 LDS 센서를 구동하는데 필요한 드라이버 패키지를 설치합니다.

### 센서 연결포트 권한 설정
```bash
$ sudo chmod a+rw /dev/ttyUSB0
```
센서를 USB로 Linux가 설치된 PC와 연결할 경우 USB 포트의 권한을 올바르게 설정해 주어야 할당된 포트에 접근할 수 있습니다.  
위 명령어는 ttyUSB0 포트에 읽기와 쓰기 권한을 설정합니다.


### hlds_laser_publisher 노드 실행
```bash
$ roslaunch hls_lfcd_lds_driver hlds_laser.launch
```
{% capture capture00 %}
**roslaunch hls_lfcd_lds_driver hlds_laser.launch**
1. hlds_laser_publisher
    - publish : scan, rpms

hlds_laser.launch를 실행하면 hlds_laser_publisher 노드가 생성되어 센서의 데이터와 회전속도를 각각 scan과 rpms의 topic으로 publish합니다. sensor_msgs타입의 LaserScan 메세지인 scan에는 LDS 센서가 회전하며 획득한 로봇의 주변 물체와의 거리 데이터가 배열의 형태로 누적되어 저장됩니다.
{% endcapture %}
<div class="notice--success">{{ capture00 | markdownify }}</div>

### RVizとhlds_laser_publisher 노드 실행
```bash
$ roslaunch hls_lfcd_lds_driver view_hlds_laser.launch
```

{% capture capture01 %}
**roslaunch hls_lfcd_lds_driver view_hlds_laser.launch**
1. hlds_laser_publisher
    - publish : scan, rpms

2. rviz
    - subscribe : scan

view_hlds_laser.launch를 실행하면 hlds_laser.launch 파일과 rviz 노드가 실행됩니다.  
hlds_laser.launch 파일의 hlds_laser_publisher 노드가 생성되어 센서의 데이터와 회전속도를 각각 scan과 rpms의 topic으로 publish합니다.  
sensor_msgs타입의 LaserScan 메세지인 scan에는 LDS 센서가 회전하며 획득한 로봇의 주변 물체와의 거리 데이터가 배열의 형태로 누적되어 저장됩니다.  
RViz 노드에서는 rviz의 설정파일을 읽어들여 화면을 띄우고 scan 데이터를 subscribe하여 3차원 그래픽으로 시각화합니다.
{% endcapture %}
<div class="notice--success">{{ capture01 | markdownify }}</div>

## 센서 소스코드 다운로드

다운로드된 LDS-01을 지원하는 드라이버는 Windows, Linux, MacOS의 개발환경에서 실행할 수 있습니다.  
소프트웨어 요구사항은 아래와 같습니다.

- GCC(Linux와 MacOS 사용시), MinGW(Windows 사용시)
- Boost library(v1.66.0에서 테스트됨)


1. 아래의 GitHub 주소에서 소스코드를 다운로드합니다.
  ```bash
  $ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
  ```
  또는, 아래의 사이트에서 초록색 `Clone or download` 버튼을 눌러 소스코드를 다운로드 할 수도 있습니다.

  - [https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)

2. 개발환경에 필요한 소프트웨어와 라이브러리를 설치합니다.
  - GCC(Linux, MacOS) 또는 MinGW(Windows)
  - Boost library

### 소스코드 빌드하기
아래의 명령어는 Linux 환경에 맞도록 설정된 makefile을 이용하여 소스코드를 빌드합니다.  
Windows와 MacOS 환경의 경우 makefile을 적절히 수정하시기 바랍니다.

```bash
$ cd hls_lfcd_lds_driver/applications/lds_driver/
$ make
```

### CLI 환경에서 실행하기
소스코드가 성공적으로 빌드되면 아래와 같이 `./lds_driver` 파일을 실행해서 LDS 센서의 값을 확인할 수 있습니다.

```bash
$ ./lds_driver
```
```
r[359]=0.438000,r[358]=0.385000,r[357]=0.379000,...
```

### GUI 환경에서 실행하기
LDS 센서의 값을 시각적으로 확인하기 위해서는 Qt Creator와 Qt Libs를 추가로 설치해야 합니다.
- Qt Creator(v4.5.0에서 테스트됨)
- Qt Libs(v5.10.0에서 테스트됨)

1. 아래 Qt 사이트에서 Open Source 버전을 설치하세요
  - [https://www.qt.io/download](https://www.qt.io/download)

2. Qt Creator를 실행합니다.
3. 소스코드에서 아래 위치의 `lds_polar_graph.pro` 파일을 엽니다. 
  (hls_lfcd_lds_driver/applications/lds_polar_graph/lds_polar_graph.pro)
4. 소스코드에서 센서와 연결된 포트를 확인하고, 다른 포트에 할당되었다면 해당 포트 번호로 변경합니다.
5. `CTRL` + `SHIFT` + `B`를 눌러 소스코드를 빌드합니다.
6. 빌드가 성공적으로 완료되면 `CTRL` + `R`을 눌러 프로그램을 실행합니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/appendix_lds/lds_gui.png)

### Embedded 보드에서 실행하기

LDS-01 센서는 OpenCR 또는 아두이노 보드에서 동작시켜볼 수 있습니다.  이 경우, 센서 데이터의 시각화를 위한 LCD 패널이 필요합니다.  
LDS-01 센서의 TX, RX 케이블은 embedded 보드의 UART 핀과 호환되며, embedded 보드에 전원과 TX, RX 핀을 연결하여 사용할 수 있습니다. 실제 케이블의 색상은 아래 그림과 다를 수 있으므로 반드시 제품 데이터시트를 참고하시기 바랍니다.。

![](/turtlebot3_textbook/assets/images/turtlebot3_textbook/lds_lines.png)

#### OpenCR에서 LDS센서 읽어오기
OpenCR 보드에서 센서값을 읽어오려면 Arduino 예제를 OpenCR 보드에 업로드 해야합니다.

1. Arduino IDE의 Tools > Board > Boards Manager 에서 OpenCR을 검색해서 라이브러리를 설치합니다.
2. Tools > Board에서 OpenCR을 선택합니다.
3. Tools > Port에서 OpenCR이 연결된 포트를 선택합니다.
4. File > Examples > OpenCR > Etc > LDS > drawLDS 예제를 선택해서 OpenCR에 업로드합니다.

예제가 성공적으로 업로드되면 OpenCR과 연결된 LCD에 아래 이미지와 같이 센서의 값이 시각적으로 보여지게 됩니다.

![](/turtlebot3_textbook/assets/images/turtlebot3_textbook/011.png)

# SLAM

SLAM(Simultaneous Localization and Mapping)이란 알려지지 않은 지역을 탐험하며 로봇에 부착된 센서를 통해 취득된 정보를 통해 로봇이 탐험하는 위치에 대한 지도를 작성하는 것을 의미합니다.  SLAM은 내비게이션이나 무인 자동차의 자율 주행에 없어서는 안될 중요한 기술입니다. 외부 정보를 취득하기 위해 사용되는 센서에는 거리를 측정할 수 있는 센서나 주변의 이미지를 얻어올 수 있는 센서들이 있습니다. 적외선 거리 센서(IR), 음파 센서(SONAR), 레이저 센서(LRF) 등이 많이 사용되며, 최근에는 영상을 해석하는 알고리즘의 발전으로 카메라가  많이 사용되기도 합니다.

로봇의 위치를 예측하기 위해 로봇 바퀴와 연결된 엔코더의 값을 읽어 주행 거리(odometry)를 추측 항법(Dead Reckoning)을 사용해서 계산하게 되는데, 이때 바퀴와 지면간의 마찰 등으로 오차가 발생하게 됩니다. 실제 로봇의 위치와 예측된 로봇의 위치의 오차를 줄이기 위해서 관성 측정 센서(IMU)로부터 얻어진 데이터를 이용해 위치를 보정할 수 있습니다.  
또한, 거리 센서의 값을 이용해 위치 오차를 감소시키는데 사용되는 방법으로는 Kalman filter, Markov Localization, Monte Carlo Localization 등이 있습니다.

SLAM으로 맵을 생성할 때에는 몇가지 주의할 점이 있습니다.  
넓은 창고나 강당과 같이 센서의 범위가 닿지 않는 넓은 영역을 지도로 만드는 경우 지도가 제대로 그려지지 않습니다. 이는 마치 센서의 범위를 두 팔의 길이라고 가정했을 때, 강당의 한 가운데에서 눈을 감고 두 팔을 이용해서 현재위치를 찾으려는 것과 같습니다.  
이와 비슷하게 특징점이 없는 긴 복도 역시 지도로 그려지기 어렵습니다. 특징이 없는 양쪽 벽면으로 이루어진 긴 복도에서 눈을 감고 벽을 손으로 짚으며 걸었을 때 복도의 어느 위치까지 왔는지 알기 힘든 것과 같습니다.  
이러한 경우 맵핑 알고리즘이 특징점으로 참고할 수 있도록 지도상의 곳곳에 물체나 장애물을 설치하는 방법을 고려해볼 수 있습니다. 이러한 특징점은 일정한 패턴이나 대칭의 형태로 놓여지는 것 보다 패턴이 없는 형태로 놓여지는 것이 가장 바람직합니다.

일반적으로 흔하게 사용되는 Gmapping과 Catographer는 지도를 생성하는 방식에 있어 약간의 차이가 있습니다. map을 바로 publish하는 Gmapping과는 달리 Catographer는 submap을 publish하고 이 submap을 모아서 map을 생성하게 됩니다. 이로 인해 서로 연결된 넓은 지역의 지도를 생성하고자 할 때에는 Catographer가 조금 더 정확한 형태의 지도를 만드는데 도움이 될 수 있습니다.

## SLAM 실행하기
1. [Remote PC] roscore를 실행합니다.  
  ```bash
  $ roscore
  ```

2. [Turtlebot PC] TurtleBot3을 구동하기 위한 기본 패키지들을 실행합니다.  
  ```bash
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```

3. [Remote PC] 새 터미널 창을열고 SLAM을 실행합니다. 아래 커맨드에서 `${TB3_MODEL}` 위치에 TurtleBot3의 모델 이름 중 하나로 바꿔적어야 합니다. 사용중인 로봇에 맞게 `burger`, `waffle`, `waffle_pi` 중 선택할 수 있습니다.
  ```bash
  $ export TURTLEBOT3_MODEL=${TB3_MODEL}
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
{% capture capture02 %}
**roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping**
1. **roslaunch turtlebot3_bringup turtlebot3_remote.launch**
  - urdf : Unified Robot Description Format의 약자로써 로봇의 구성과 연결형태를 표현하는 XML 형식의 파일입니다.
  - robot_state_publisher : robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf의 형식으로 publish 합니다.
  - subscribe : joint_states 
  - publish : tf

    turtlebot3_remote.launch 파일을 실행시키면 로봇의 urdf를 정의된 위치에서 불러옵니다. 또한, joint_states와 urdf를 이용하여 tf를 publish하는 robot_state_publisher 노드를 생성합니다.  
    turtlebot3_slam.launch 파일 내부에 turtlebot3_remote.launch를 포함하기 때문에 turtlebot3_slam.launch가 실행되면 자동적으로 turtlebot3_remote.launch가 가장 먼저 실행됩니다.

2. **turtlebot3_gmapping.launch**
  - subscribe : scan, tf
  - publish : map, map_metadata

    실행문의 끝에서 slam_methods:=gmapping이라는 옵션을 입력했기 때문에 gmapping의 실행 관련된 파일인 turtlebot3_gmapping.launch가 실행됩니다. 이 파일 내부에서는 다시 LDS의 설정이 저장된 turtlebot3_lds_2d.lua를 불러오며 gmapping의 사용을 위해 필요한 각종 파라미터를 정의하고 gmapping 패키지의 slam_gmapping 노드를 실행합니다. slam_gmapping 노드가 생성되면 scan과 tf 토픽을 subscribe하여 맵을 생성하는데 필요한 map_metadata와 map을 publish합니다.

3. **rviz**
  - subscribe : tf, scan, map

    마지막으로 rviz의 설정파일을 적용한 rviz가 실행되어 tf, scan, map데이터를 subscribe하여 로봇과 센서값, gmapping을 통해 생성된 맵을 시각화합니다.  
{% endcapture %}
<div class="notice--success">{{ capture02 | markdownify }}</div>

**참고**  
위의 명령을 실행했을 때 SLAM이 Rviz 화면에서 제대로 실행되지 않는 경우 Remote PC와 TurtleBot3 SBC에서 다음 명령을 실행하여 시스템 시간을 동기화 하시기 바랍니다.
**$ sudo ntpdate ntp.ubuntu.com**
{: .notice}

### TurtleBot3는 다양한 방식의 SLAM을 지원합니다. 
Gmapping, cartographer와 같은 여러 SLAM을 지원하며, 커맨드상에서 `slam_methods`의 파라미터 값으로 직접 입력함으로써 선택할 수 있습니다.  
입력 가능한 옵션으로는 `gmapping`, `cartographer`, `hector`, `karto`, `frontier_exploration`가 있습니다.  
예를 들어, Gmapping 대신 Google사의 cartographer를 SLAM으로 사용하고자 한다면 아래와 같이 slam_methods의 인자값으로 cartographer를 넘겨주어 SLAM 노드를 실행시킬 수도 있습니다.

```bash
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
```

{% capture capture03 %}
**roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer**
1. roslaunch turtlebot3_bringup turtlebot3_remote.launch
  - urdf : Unified Robot Description Format의 약자로써 로봇의 구성과 연결형태를 표현하는 XML 형식의 파일입니다.
  - robot_state_publisher : robot_state_publisher에서는 로봇의 각 관절에 대한 정보를 수신하고 얻어진 관절에 대한 정보를 urdf를 참고해서 tf의 형식으로 publish 합니다.
  - subscribe : joint_states 
  - publish : tf

    turtlebot3_remote.launch 파일을 실행시키면 로봇의 urdf를 정의된 위치에서 불러옵니다. 또한, joint_states와 urdf를 이용하여 tf를 publish하는 robot_state_publisher 노드를 생성합니다.  
    turtlebot3_slam.launch 파일 내부에 turtlebot3_remote.launch를 포함하기 때문에 turtlebot3_slam.launch가 실행되면 자동적으로 turtlebot3_remote.launch가 가장 먼저 실행됩니다.

2. turtlebot3_cartographer.launch
  - subscribe : scan, imu, odom
  - publish : submap_list, map
  
    실행문의 끝에서 slam_methods:=cartographer라는 옵션을 입력했기 때문에 cartographer의 실행 관련된 파일인 turtlebot3_cartographer.launch가 실행됩니다.  
    이 파일 내부에서는 다시 LDS의 설정이 저장된 turtlebot3_lds_2d.lua를 불러오며 cartographer의 사용을 위해 필요한 각종 파라미터를 정의하고 cartographer_ros 패키지의 cartographer_node 노드를 실행하여 scan, imu, odom 등의 topic을 subscribe하고 submap_list를 publish 합니다.  
    또한, cartographer_ros 패키지의 cartographer_occupancy_grid_node 노드를 실행하여 cartographer_node에서 publish된 submap_list를 subscribe하여 map을 publish합니다.

3. rviz
  - subscribe : tf, scan, map

    마지막으로 rviz의 설정파일을 적용한 rviz가 실행되어 tf, scan, map데이터를 subscribe하여 로봇과 센서값, cartographer를 통해 생성된 맵을 시각화합니다.
{% endcapture %}
<div class="notice--success">{{ capture03 | markdownify }}</div>

기본으로 제공하는 Gmapping이 아닌 다른 SLAM을 사용하려면 아래와 같이 관련 패키지를 설치해야 합니다.

- cartographer 설치 : 현재 최신 버전의 cartographer (v1.0.0)은 2018년 이후로 업데이트 되지 않았으며 TurtleBot3 패키지로 시뮬레이션을 시도할 때 정상적으로 구동되지 않을 수 있습니다. TurtleBot3의 SLAM을 시뮬레이션 할 때에는 Gmapping을 사용하시기 바랍니다.

다음의 설치 방법은 ROS1 Kinetic에서만 사용할 수 있습니다.  
  ```bash
  $ sudo apt-get install ninja-build libceres-dev libprotobuf-dev protobuf-compiler libprotoc-dev
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/googlecartographer/cartographer.git
  $ git clone https://github.com/googlecartographer/cartographer_ros.git
  $ cd ~/catkin_ws
  $ src/cartographer/scripts/install_proto3.sh
  $ rm -rf protobuf/
  $ rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:xenial
  $ catkin_make_isolated --install --use-ninja
  $ source ~/catkin_ws/install_isolated/setup.bash
  ```
- Hector Mapping 설치하기
  ```bash
  $ sudo apt-get install ros-kinetic-hector-mapping
  ```
- Karto 설치하기
  ```bash
  $ sudo apt-get install ros-kinetic-karto 
  ```
- Frontier Exploration 설치하기
  ```bash
  $ sudo apt-get install ros-kinetic-frontier-exploration ros-kinetic-navigation-stage
  ```

SLAM 노드가 구동하는 경우, 다음과 같이 다양한 방식의 SLAM을 적용한 시각화 도구 인 RViz을 별도로 수행 할 수 있습니다. 이미 실행되고있는 RViz가있는 경우, 프로그램 충돌이 발생할 수 있습니다.

```bash
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_gmapping.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_cartographer.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_hector.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_karto.rviz
$ rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_frontier_exploration.rviz
```

## 원격 제어 노드 실행하기
[Remote PC] SLAM으로 지도를 만들기 위해서는 탐색되지 않은 지도영역의 위치로 로봇을 이동시켜야 합니다. 이를 위해 TurtleBot3를 원격으로 제어 할 수있는 노드를 다음과 같이 실행합니다.

SLAM으로 지도를 만들 때는 지나치게 격렬한 움직임이나 갑작스러운 방향 전환 등을 피할수록 정확한 지도 작성에 도움이 됩니다. RViz에 그려진 지도를 보면서 미완성된 영역으로 로봇을 이동해서 지도를 완성시켜야 합니다.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

```
Control Your TurtleBot3!
---------------------------
Moving around:
        w
    a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/slam_running_for_mapping.png)


## 튜닝 가이드
Gmapping은 다양한 환경에 최적화된 성능을 구현하기 위해 여러 파라미터를 설정할 수 있습니다. Gmapping은 일반적으로 별도의 설정 없이 사용이 가능하며, 기본 설정으로 사용하는 경우가 많습니다. 설정 가능한 파라미터의 목록은 [ROS wiki의 Gmapping 파라미터](http://wiki.ros.org/gmapping#Parameters) 페이지를 참고하시기 바랍니다.

다음의 파라미터들은 `turtlebot3_slam/launch/turtlebot3_gmapping.launch` 파일에 정의되어 있으며, 파일을 실행할 때 roscore의 파라미터 서버에 로딩되어 gmapping을 기반으로 하는 SLAM에 적용됩니다.

### maxUrange
이 매개 변수는 LDS 센서의 최대 거리를 설정합니다. 
 
### map_update_interval 
맵을 업데이트하는 시간(초)이 값이 낮을수록 지도가 더 자주 업데이트됩니다. 그러나 그만큼 더 큰 부하가 지도 계산에 필요합니다. 환경에 따라이 매개 변수를 설정하십시오.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/tuning_map_update_interval.png)

### minimumScore 
센서의 scan 데이터 일치 검사의 성공과 실패를 결정하는 최소 점수값을 설정합니다. 넓은 공간에서 로봇의 예상위치에 오차가 생기는 것을 완화할 수 있습니다. 적절하게 설정된 경우 아래와 같은 정보를 볼 수 있습니다. 

```
Average Scan Matching Score=278.965
neff= 100
Registering Scans:Done
update frame 6
update ld=2.95935e-05 ad=0.000302522
Laser Pose= -0.0320253 -5.36882e-06 -3.14142
```
이 값이 너무 높으면 아래와 같은 경고를 볼 수 있습니다. 
```
Scan Matching Failed, using odometry. Likelihood=0
lp:-0.0306155 5.75314e-06 -3.14151
op:-0.0306156 5.90277e-06 -3.14151
```

### linearUpdate
로봇이 이 값보다 더 긴 거리를 병진운동 했을 때 scan 프로세스를 실행합니다.
 
### angularUpdate 
로봇이 이 값보다 더 큰 각도를 회전운동 했을 때 scan 프로세스를 실행합니다. 
이 값을 linearUpdate보다 작게 설정하는 것이 좋습니다.

## 지도 저장하기
[Remote PC] 지도가 완성되면 map_saver 노드를 실행해서 생성된 맵을 저장해야 합니다. 로봇이 움직일 때 발생한 주행거리와 tf, scan 데이터 등을 통해 RViz 상에서 완성된 지도는 아래의 명령어를 통해 파일로 저장될 수 있습니다.
완성된 지도는 2개의 파일로 나누어 저장되고 그 중 pgm는 Portable Gray Map 형식의 이미지 파일이며, yaml는 지도의 해상도 등 각종 설정을 저장하는 파일입니다.

```bash
$ rosrun map_server map_saver -f ~/${map_name}
```

`-f` 옵션은 지도 파일이 저장될 위치와 파일명을 지정합니다. 위 명령어에서는 `~/map` 옵션이 사용되었기 때문에 사용자의 home 폴더(`~/` 또는 `/home/\<username\>`)에 `map.pgm`와`map.yaml` 파일로 저장됩니다.

## 지도
ROS에서 지도는 2차원 Occupancy Grid map(OGM)을 주로 사용합니다. 저장된 map.pgm 이미지파일을 열어보면 아래와 같이 로봇이 이동할 수 있는 흰색 영역과, 장애물로 식별되어 로봇이 이동할 수 없는 검정색 영역, 로봇이 탐험하지 않은 회색영역으로 구분됩니다. 이렇게 생성된 맵은 다음에 배울 Navigation에서 사용될 수 있습니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/map.png)

아래의 이미지는 cartographer를 이용해서 광대한 영역의 지도를 생성한 예시입니다. 아래와 같은 맵을 생성하는 데에는 약 한시간 정도의 시간동안 총 350m의 거리를 로봇을 조종해서 만들 수 있었습니다.

![](http://emanual.robotis.com/assets/images/platform/turtlebot3/slam/large_map.png)
