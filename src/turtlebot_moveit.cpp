#include <memory>
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "turtlebot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("turtlebot_moveit");
  RCLCPP_INFO(logger, "Hello, MoveIt!"); // Use RCLCPP_INFO for logging

  // moveit core 의 arm 그룹을 사용하기위한 선언
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_interface = MoveGroupInterface(node, "arm");
  arm_interface.setPlanningPipelineId("move_group");
  arm_interface.setPlanningTime(10.0); 

  // moveit core 의 gripper 그룹을 사용하기위한 선언
  using moveit::planning_interface::MoveGroupInterface;
  auto gripper_interface = MoveGroupInterface(node, "gripper");

  // 로봇 상태를 가져오기 위해 로봇 상태 모니터를 설정
  auto robot_state = arm_interface.getCurrentState(10.0);
  if (!robot_state) {
    RCLCPP_ERROR(logger, "Failed to fetch current robot state");
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;

  RCLCPP_INFO(node->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
  

  // 좌표를 통한 moveit 이동
  auto const target_pose = [current_pose]{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = current_pose.orientation.w;
    msg.orientation.x = current_pose.orientation.x;
    msg.orientation.y = current_pose.orientation.y;
    msg.orientation.z = current_pose.orientation.z;
    msg.position.x = current_pose.position.x;
    msg.position.y = current_pose.position.y;
    msg.position.z = current_pose.position.z-0.03; //-> z값을 -3cm
    return msg;
  }();
  arm_interface.setPoseTarget(target_pose);

  // Plan 을 생성
  auto const [success, plan] = [&arm_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // plan 이 생성되면 execute
  if(success) {
    RCLCPP_INFO(logger, "Planning Success!, execute the plan");
    arm_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Gripper 를 조작 "open"
  gripper_interface.setNamedTarget("open");
  gripper_interface.move();

  //gripper 를 조작 "close"
  gripper_interface.setNamedTarget("close");
  gripper_interface.move();

  //Name Target 을 통한 arm 이동
  //Name Target 은 turtlebot3_moveit_config/config/turtlebot3.srdf 에 정의되어 있음
  // <group_state name="home" group="arm">
  //       <joint name="joint1" value="0"/>
  //       <joint name="joint2" value="-1"/>
  //       <joint name="joint3" value="0.7"/>
  //       <joint name="joint4" value="0.3"/>
  //   </group_state>
  // group_state 의 name 을 을 변경하고 joint state 를 추가하여 사용 가능
  // group="gripper" 로 설정하면 gripper 도 사용 가능
  
  arm_interface.setNamedTarget("home");
  arm_interface.move();

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}