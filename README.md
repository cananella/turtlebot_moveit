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


# camera node and calibration
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