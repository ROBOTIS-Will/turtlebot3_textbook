---
layout: splash
lang: kr
ref: turtlebot3_textbook4
permalink: /docs/turtlebot3_textbook/week4/
sidebar:
  title: TB3 TextBook
  nav: "turtlebot3_textbook4"
---

# Machine Learning

## Machine Learning I : object_detector_3d

### 목표
Machine Learning 프레임 워크 중 하나인 chainer를 이용하여 물체를 인식하고 Depth camera와 연동하여 그 물체와의 거리를 구해본다. (링크 : [chainer](https://chainer.org/))


### 동작환경
- Ubuntu 16.04
- ROS Kinetic
- Python 2.7.16
- Intel RealSense D435


### 설정(Setup)
1. ROS Kinetic 설치 : [wiki.ros.org](http://wiki.ros.org/kinetic/Installation/Ubuntu)를 참고하세요.

2. RealSense D435 ROS 패키지 설치
    ```
    $ sudo apt install ros-kinetic-realsense2-camera
    ```

3. 의존 패키지 설치
    - pip가 없는 경우 설치
        ```
        $ sudo apt install python-pip
        ```
    - chainer, chainercv
        ```
        $ pip install chainer chainercv
        ```
    - ros_numpy
        ```
        $ sudo apt install ros-kinetic-ros-numpy
        ```

4. object_detector_3d 설치
    - catkin workspace로 이동
        ```
        $ cd ~/catkin_ws/src
        ```
    - github lfs 설치
        ```
        $ curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
        $ sudo apt install git-lfs
        ```
    - object_detector_3d 소스, model 다운로드(github lfs 사용)
        ```
        $ git clone https://github.com/NobutakaShimada/object_detector_3d.git
        ```
    - 컴파일
        ```
        $ cd ~/catkin_ws
        $ catkin_make
        ```
 
### 실행(PC에 따라 다르나 실행하는데 몇 초 이상이 걸림)
1. realsense & object_detector_3d node
    ```
    $ roslaunch object_detector_3d run.launch
    ```
2. rviz
    ```
    $ roslaunch object_detector_3d rviz.launch
    ```


### 실행화면
![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/week4_01.png)

> 그림의 왼쪽 : rostopic echo /object_detection_3d 에 의한 출력이 표시 되고 있다.

> 그림의 오른쪽 상단 : rviz를 통해 /object_detection_3d/result_image가 표시되고 있다.

> 오른쪽 상단 화면을 보면 앞에서부터 키보드, 머그컵, 병, 2대의 모니터가 각각 검출되는 것을 알 수 있다.  각각의 탐지마다 캡션이 붙어 있어 다음 정보가 표시되고 있다.

  - 물체의 이름
  - 검출 점수. 구간 [0, 1]에 포함된 수치로 1에 가까울 수록 신뢰도가 높음을 의미한다.
  - 물체의 중심점(후술)의 3차원 좌표. 좌표계의 원점은 카메라의 중심이며 x, y, z축 방향은 각각 오른쪽, 아래, 안쪽 방향으로 단위는 미터이다.

5개의 검출 결과 중 깊이 방향의 거리인 z값을 보면, 물체의 위치가 앞에서 안쪽에 있음에 따라 값이 커지고 있는 것을 확인 할 수 있다.

### ROS 노드
1. Topic
    - Subscribed Topics
        - /camera/color/image_raw [sensor_msgs/Image]  
        컬러 이미지, 2D 객체 검출에 이용

        - /camera/depth/color/points [sensor_msgs/PointCloud2]  
        3D PointCloud, 3차원 위치를 구하기 위해 사용
        사용시 위 camera image와 시간 동기화가 필요하다.

    - Published Topics
        - /object_detection_3d [object_detector_3d/Detection3DResult]
            ```
            int32 num_detections
            Detection3D[] detections
            ```
        이 토픽은 검출한 물체의 수(num_detections)와 검출 정보(detections)로 구성되어 있다. 
        검출 정보인 Detection3D는 아래와 같은 정보로 이루어져 있다.
        ```
        int32 class_id
        string class_name
        float32 score
        float32 y_min
        float32 x_min
        float32 y_max
        float32 x_max
        geometry_msgs/Point position
        ```

        class_id, class_name은 검출된 물체의 분류 번호와 이름이며, score는 검출의 신뢰도를 의미한다. y_min, x_min, y_max, x_max는 검출된 물체의 경계박스(bounding box)의 왼쪽 상단과 오른쪽 하단의 좌표이다. 마지막으로 position은 물체의 3차원 위치이다.

        - /object_detection_3d/result_image [sensor_msgs/Image]  
        검출에 사용된 이미지에 검출된 물체의 정보가 포함된 결과 이미지이다. 상기의 오른쪽과 같은 이미지이다.

    - 기타  
        - sensor_msgs.CameraInfo: /camera/color/image_raw의 내부 파라미터 
        - realsense2_camera.Extrinsics: 점군의 좌표계에서 컬러 카메라 좌표계로 변환하기 위한 외부 매개변수


### 구현 세부사항 (Implementation details)
1. 입출력 데이터
    - 입력
        - 2D 카메라 이미지
        - 3D PointCloud
        - 카메라의 내부 파라미터
        - PointCloud 좌표계에서 카메라의 위치를 나타내는 외부 매개변수

    - 출력
        - PointCloud 좌표계에서 물체의 중심점의 좌표
        - 물체의 종류
        - 검출(Detection)의 신뢰도

2. 알고리즘의 개요  
다음 단계에 따라 물체의 3D 좌표를 추출
    - (1) 카메라에서 얻은 이미지를 입력으로 물체 검출기(Object Detector)를 이용하여 이미지에서 복수의 물체를 검출한다.
    - (2) 각 검출에 대응하는 bbox(bounding box)에 대한 PointCloud 좌표계에서 절두체(frustum)를 구한다.
    - (3) 각 검출에 대응하는 시각 절두체(view frustum)에 대해 그 안에 포함된 point의 부분집합을 추출한다.
    - (4) 각 point 부분집합 그룹에 대한 중심점의 좌표를 구한다.
    - (5) 2D 검출 결과와 3D 중심점 좌표를 통합하여 3D 검출 결과로 만든다.

3. 각 알고리즘에 대한 설명
    - (1) 2D 물체 검출(2D Object detection)  
    2D 물체 검출기는 이미지에 포함 된 (미리 정의 된 종류의) 물체를 감지하는 것을 말한다. 검색은 2D 이미지를 입력하면 다음과 같은 정보가 출력된다.  
    이는 복수의 물체의 각각에 대한 예측값이다.
        - 물체를 둘러싼 박스（axis aligned bounding box, bbox）
        - 물체의 종류
        - 예측의 신뢰도	

        구체적으로 사용하는 물체 검출기(Object Detector)는 [MS COCO](http://cocodataset.org/)데이터 세트에 의해 학습된 [SSD300](https://arxiv.org/abs/1512.02325)을 이용하였으며, 이 물체 검출기(Object Detector)는 CNN을 이용해 80 종의 물체를 검출하게 된다.  
        구체적인 검출 항목은 링크되어 있는 목록을 참고하기 바란다.  
        구현은 python의 deep learning framework인 [chainer](https://github.com/chainer/chainer)를 사용하고 있으며 특히 이미지 처리 관련해서는 [chainercv](https://github.com/chainer/chainercv)를 이용한다.
        
    - (2, 3) 검출한 물체의 point 부분집합 추출
    여기서는 이전 단계에서 얻은 bbox 정보를 이용한다. 개별 bbox에 대해 PointCloud의 모든 point 중 카메라 시점에서 보아 bbox 안에 들어가는 점만 부분 PointCloud로 추출하고 있다. 

    - (4) 부분 PointCloud에서의 중심점 계산  
    부분 PointCloud는 대상 물체와 배경 및 차폐된 물체들 로부터 얻어진 point들로 구성된다. 이러한 점들을 대표하는 하나의 집약된 점을 구하게 되는데 이를 중심점이라 한다.  
    중심점의 정의에는 여러 가지가 있지만, 소프트웨어는 부분 PointCloud의 중심을 중심점으로 정의한다.  
    그러나 이 방법은 추출한 물체와 물체가 아닌 부분을 구분하지 않고 점 데이터로 이용하고 있기 때문에 물체의 형상이나 bbox의 차이 등에 따라 물체 자체의 중심에서 떨어진 위치가 중심점으로 산출되어 버릴 가능성이 있다.  
    추출한 물체가 아닌 부분은 부분 PointCloud에서 제거 한 후 남은 PointCloud로 중심을 계산하는게 더 나은 중심점의 정의로 보여진다

    - (5) 2D 검출과 3D 중심점 좌표의 통합   
    단순하여 생략한다.


## Machine Learning II: YOLO

### 목표
ROS 환경에서 YOLO를 사용하여 물체를 인식하는 것을 따라해 본다. YOLO(You Only Look Once)는 실시간 물체 탐색 시스템으로 타 물체인식기에 비해 빠른 속도를 자랑한다. YOLO는 DNN(deep neural network)들을 학습시키고 실행하는 신경망 프레임워크(neural network framework)인 darknet을 이용해 구동한다.

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/week4_02.png)

### 동작환경
- Ubuntu 16.04
- ROS Kinetic
- Intel RealSense D435


### 설정(Setup)
- ROS Kinetic 설치 : [wiki.ros.org](http://wiki.ros.org/kinetic/Installation/Ubuntu)를 참고하세요.

- RealSense D435 ROS 패키지 설치  
    ```
    $ sudo apt install ros-kinetic-realsense2-camera
    ```

- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)(ROS용 YOLO) 설치
    - catkin workspace로 이동
        ```
        $ cd ~/catkin_ws/src
        ```
    - 소스 다운로드
        ```
        $ git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
        ```
    - 컴파일
        ```
        $ cd ~/catkin_ws
        $ catkin_make -DCMAKE_BUILD_TYPE=Release
        ```
     - yolo v3용 weight 다운로드
        ```
        $ cd ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
        $ wget http://pjreddie.com/media/files/yolov3.weights
        ```
    - darknet_ros 설정 변경
        - darknet_ros/config/ros.yaml 파일을 편집기로 연다.
            ```
            $ nano ~/catkin_ws/src/darknet_ros/darknet_ros/config/ros.yaml
            ```
        - camera_reading에 있는 topic을 /camera/color/image_raw 로 변경한 후 저장한다.
            ```
            subscribers:

            camera_reading:
                topic: /camera/color/image_raw
                queue_size: 1
            ```

### 실행(미리 학습된 모델 이용)
1. realsense
    ```
    $ roslaunch realsense2_camera rs_camera.launch
    ```

2. YOLO(darket_ros)
    ```
    $ roslaunch darknet_ros darknet_ros.launch
    ```

### 실행화면

![](https://nobutakashimada.github.io/ritsumeikan_github/assets/images/ritsumeikan/week4_03.png)

> 위의 사진처럼 여러개의 물체가 동시에 인식이 되며 인식된 물체의 경계에 박스가 생기며 물체의 이름이 박스 좌상단에 위치하게 된다. 


### ROS 노드
1. Topic
    - Subscribed Topics
        - /camera/color/image_raw [sensor_msgs/Image]  
        컬러 이미지, 물체 검출에 이용

    - Published Topics
        - /darknet_ros/bounding_boxes [darknet_ros_msgs/BoundingBoxes]  
        인식한 물체의 정보를 담고 있는 topic으로 아래와 같이 메시지의 header, 검출에 사용한 이미지의 header, 검출한 물체 정보인 BoundingBox로 구성되어 있다.
            ```
            Header header
            Header image_header
            BoundingBox[] bounding_boxes
            ```
            물체 정보를 나타내는 BoundingBox는 아래와 같다.(BoundingBox.msg)  
            ```
            float64 probability
            int64 xmin
            int64 ymin
            int64 xmax
            int64 ymax
            int16 id
            string class
            ```
            검출의 정확도를 나타내는 probability, 검출한 물체를 나타내는 이미지상 경계박스의 x, y 위치, 물체의 구분번호인 id, 물체의 종류를 나타내는 class로 구성되어 있다.

        - /darknet_ros/detection_image  [sensor_msgs/Image]  
        검출에 사용된 이미지에 검출된 물체의 정보가 포함된 결과 이미지이다.

        - /darknet_ros/found_object [std_msgs/Int8]  
        검출된 물체의 개수를 표시

2. Actions
    - camera_reading [sensor_msgs::Image]  
    이미지와 결과값(검출된 물체의 경계박스들)이 포함된 액션을 보낸다.

3. Parameters   
검출과 관련된 파라미터 세팅은 `darknet_ros/config/yolo.yaml` 과 유사한 이름의 파일에서 할 수 있다. ROS와 연관된 파라미터에 대한 세팅은 `darknet_ros/config/ros.yaml` 파일에서 할 수 있다.

    - image_view/enable_opencv (bool)  
        bounding box가 포함된 검출 이미지를 보여주는 open cv viewer를 끄고 켬.
    - image_view/wait_key_delay (int)  
        open cv viewer에서 wait key delay(ms)
    - yolo_model/config_file/name (string)  
        검출에 사용하는 네트워크의 cfg 이름. 프로그램은 darknet_ros/yolo_network_config/cfg 폴더에서 이름에 맞는 cfg 파일을 불러와 사용한다.
    - yolo_model/weight_file/name (string)  
        검출에 사용하는 네트워크의 weight 파일이름. 프로그램은 darknet_ros/yolo_network_config/weights 폴더에서 이름에 맞는 weights 파일을 불러와 사용한다. 
    - yolo_model/threshold/value (float)    
        검출 알고리즘의 threshold, 0과 1 사이의 값이다.
    - yolo_model/detection_classes/names (array of strings) 
        네트워크가 검출 가능한 물체들의 이름

### GPU 가속
NVidia GPU를 가지고 있는 경우 CUDA를 이용하면 CPU만 사용하는 경우보다 몇배 빠른 검출이 가능하다. CUDA를 설치 했다면 CMakeLists.txt 파일에서 자동으로 인식하여 컴파일(catkin_make)시에 GPU 모드로 컴파일 된다.
- [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit)


### 참고 사이트
- [darknet](https://pjreddie.com/darknet/yolo/)
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
