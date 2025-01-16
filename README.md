# Turtlebot_manipulator control with moveit2 

1. [Turtlebot3 Quick start guide 를 참고하여 PC, SBC 셋팅을 완료한다.](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
2. [Moveit2 를 ~/[ros2_work_space]/src 안에 clone 한다.](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)
3. 본 repo 를 clone 한다.

## turtlebot test code
``` bash
## turtlebot 에서 bringup hardware
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

## PC 에서 moveit2 core 켜기
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py 

## moveit 패키지의 코드를 사용하여 로봇을 동작
ros2 run turtlebot_moveit turtlebot_moveit
```


## camera node and calibration
1. [camera node on SBC](https://github.com/ros-drivers/usb_cam)
```bash
## on SBC
ros2 run usb_cam usb_cam_node_exe
```

2. [camera calibration](https://docs.nav2.org/tutorials/docs/camera_calibration.html)
```bash
## on PC
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args -r image:=/image_raw
```


## aruco marker detect
``` bash
## on SBC  turn on cam 
ros2 run usb_cam usb_cam_node_exe   ## carmera node
ros2 run aruco_marker_detect aruco_marker_detector  ##aruco marker detector on SBC

## on pc   turn on detector
## aruco marker detector on PC -> 딜레이가 길면 원활하게 작동안될수있음
ros2 run turtlebot_moveit aruco_marker_detect.py    


## on pc    aruco detect & move
## id 가 0 인 아루코 마커가 detect 될때 z 값이 0.13m 보다 크다면 cmd_vel 에 linear_x = 0.15 를 publish 함
ros2 run turtlebot_moveit aruco_movet_test.py


```