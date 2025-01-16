#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot_cosmo_interface.srv import MoveitControl
from geometry_msgs.msg import Pose, PoseArray

class TurtlebotArmClient(Node):

    def __init__(self):
        super().__init__('turtlebot_arm_client')
        self.client = self.create_client(MoveitControl, 'moveit_control')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveitControl.Request()

    def send_request(self, cmd, posename='', waypoints=None):
        self.req.cmd = cmd
        self.req.posename = posename
        if waypoints:
            self.req.waypoints = waypoints
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = TurtlebotArmClient()

    # Example usage
    cmd = 1  # Command to move to a named target
    posename = 'home'
    response = client.send_request(cmd, posename)
    client.get_logger().info(f'Response: {response.response}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()