#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray
from srv_call_test import TurtlebotArmClient
from aruco_move_test import ArucoMarkerMoveController
import time

def main(args=None):
    rclpy.init(args=args)
    arm_client = TurtlebotArmClient()
    move_client = ArucoMarkerMoveController()

    print ("task start!")

    move_client.target_marker_id = 0
    
    while(move_client.target_marker_distance is None):
        rclpy.spin_once(move_client)
        print("waiting for marker")
        time.sleep(0.1)
        pass
    try:
        while(move_client.target_marker_distance > 0.13):
            rclpy.spin_once(move_client)
            print(move_client.target_marker_distance)    
            move_client.publish_cmd_vel(0.15)
            time.sleep(0.1)
    
    except:
        pass




    # # # Example usage
    # ##arm control
    # response = arm_client.send_request(1, "grip_center")
    # arm_client.get_logger().info(f'Response: {response.response}')
    
    # ## gripper control
    # response = arm_client.send_request(2, "open")
    # arm_client.get_logger().info(f'Response: {response.response}')


    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()