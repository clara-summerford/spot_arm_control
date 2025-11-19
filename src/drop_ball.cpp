#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include <moveit/planning_scene_interface/planning_scene_interface.h> // changed from .hpp (for spot)
#include <std_srvs/srv/trigger.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("spot_moveit");

#include <std_srvs/srv/trigger.hpp>

bool callTriggerService(const std::string &service_name,
                        rclcpp::Node::SharedPtr caller_logger)
{
    // separate node for the service client
    auto client_node = std::make_shared<rclcpp::Node>("gripper_client");
    auto client = client_node->create_client<std_srvs::srv::Trigger>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(caller_logger->get_logger(),
                     "Service '%s' not available", service_name.c_str());
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(client_node);

    if (exec.spin_until_future_complete(future)
            == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        RCLCPP_INFO(caller_logger->get_logger(),
                    "%s response: success=%d, message='%s'",
                    service_name.c_str(),
                    response->success,
                    response->message.c_str());
        return response->success;
    }

    RCLCPP_ERROR(caller_logger->get_logger(),
                 "Failed to call service '%s'", service_name.c_str());
    return false;
}

bool openGripper(rclcpp::Node::SharedPtr node){
    return callTriggerService(
        "/spot_manipulation_driver/open_gripper",
        node
    );
}

bool closeGripper(rclcpp::Node::SharedPtr node){
    return callTriggerService(
        "/spot_manipulation_driver/close_gripper",
        node
    );
}

bool stowArm(rclcpp::Node::SharedPtr node){
    return callTriggerService(
        "/spot_manipulation_driver/stow_arm",
        node
    );
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread exec_thread([&executor]() { executor.spin(); });
    const std::string planning_group = "arm";

    moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm", "robot_description", "/spot_moveit");
    moveit::planning_interface::MoveGroupInterface move_group(node, move_group_options);
    // move_group.waitForServers();

    RCLCPP_INFO(LOGGER, "Move group interface initialized...");
    
    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // --- go to mini unstow pose, moveit can't plan from stow due to collisions ---
    unstowArm(node);
    if (!unstowArm(node)) {
    RCLCPP_WARN(node->get_logger(), "Arm failed to unstow.");
    }

    // --- subscribe to nav goal pose (maybe not needed for drop) ---
    // geometry_msgs::msg::PoseStamped target_pose;
    // auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "/target_pose_topic", 10,
    //     [&target_pose](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    //     {target_pose = *msg;}
    //     );

    // --- define drop pose ---
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose drop_pose = current_pose;
    drop_pose.position.z += 0.1; // roughly 3.5 in

    move_group.setPoseTarget(drop_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to drop pose.");
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at drop pose.");
            return 1;
        }
 
    // --- define open hand pose, using open_gripper service call---
    openGripper(node);
    if (!openGripper(node)) {
    RCLCPP_WARN(node->get_logger(), "Gripper failed to open.");
    }

    // --- close hand ---
    closeGripper(node);
    if (!closeGripper(node)) {
    RCLCPP_WARN(node->get_logger(), "Gripper failed to close.");
    }   

    // --- stow arm ---
    stowArm(node);
    if (!stowArm(node)) {
    RCLCPP_WARN(node->get_logger(), "Arm failed to stow.");
    }   

    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
