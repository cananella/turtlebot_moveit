#include <rclcpp/rclcpp.hpp>
#include <turtlebot_cosmo_interface/srv/moveit_control.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_array.hpp>

class TurtlebotArmController : public rclcpp::Node {
public:
    TurtlebotArmController() : Node("turtlebot_arm_controller") {
        service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveitControl>(
            "moveit_control", std::bind(&TurtlebotArmController::handleMoveitControl, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Ready to receive MoveitControl commands.");
    }


private:
    void handleMoveitControl(const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Request> req,
                             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Response> res) {
        
        // moveit core 의 arm 그룹을 사용하기위한 선언
        using moveit::planning_interface::MoveGroupInterface;
        auto arm_interface = MoveGroupInterface(shared_from_this(), "arm");
        arm_interface.setPlanningPipelineId("move_group");
        arm_interface.setPlanningTime(10.0); 

        // moveit core 의 gripper 그룹을 사용하기위한 선언
        auto gripper_interface = MoveGroupInterface(shared_from_this(), "gripper");
        

        // 좌표를 통한 이동 아직 미구현
        if (req->cmd == 0) {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, plan.trajectory_);
            res->response = (fraction > 0.95 && arm_interface.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        else if (req->cmd == 1) {
            arm_interface.setNamedTarget(req->posename);
            res->response = (arm_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        } 

        else if (req->cmd == 2) {
            gripper_interface.setNamedTarget(req->posename);
            res->response = (gripper_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        } 

        else {
            res->response = false;
        }
    }

    rclcpp::Service<turtlebot_cosmo_interface::srv::MoveitControl>::SharedPtr service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotArmController>());
    rclcpp::shutdown();
    return 0;
}