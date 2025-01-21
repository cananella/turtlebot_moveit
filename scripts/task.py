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

    # move_client.target_marker_id = 0
    
    # while(move_client.target_marker_distance is None):
    #     rclpy.spin_once(move_client)
    #     print("waiting for marker")
    #     time.sleep(0.1)
    #     pass
    # try:
    #     while(move_client.target_marker_distance > 0.13):
    #         rclpy.spin_once(move_client)
    #         print(move_client.target_marker_distance)    
    #         move_client.publish_cmd_vel(0.15)
    #         time.sleep(0.1)
    
    # except:
    #     pass


    ### pose 기준으로 움직이기
    # pose_array = PoseArray()
    # pose = Pose()
    # pose.position.x = 0.054273
    # pose.position.y = -0.081886
    # pose.position.z = 0.15

    # pose_array.poses.append(pose)

    # response = arm_client.send_request(0, "", pose_array)
    # arm_client.get_logger().info(f'Response: {response.response}')



    # # # Example usage
    # ##arm control
    # response = arm_client.send_request(1, "home")
    # arm_client.get_logger().info(f'Response: {response.response}')
    
    # ## gripper control
    response = arm_client.send_request(2, "close")
    arm_client.get_logger().info(f'Response: {response.response}')

    response = arm_client.send_request(9, "")
    arm_client.get_logger().info(f'Response: {response.response}')
    

    print ("task end!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()