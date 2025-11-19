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

bool callTriggerService(const std::string &service_name)
                        // rclcpp::Node::SharedPtr node)
                        // rclcpp::executors::SingleThreadedExecutor &executor)
{
    // separate node for the service client
    auto temp_node = std::make_shared<rclcpp::Node>("temp_service_client");
    auto client = temp_node->create_client<std_srvs::srv::Trigger>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(rclcpp::get_logger("spot_moveit"),
                     "Service '%s' not available", service_name.c_str());
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    rclcpp::executors::SingleThreadedExecutor temp_exec;
    temp_exec.add_node(temp_node);

    if (temp_exec.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(rclcpp::get_logger("spot_moveit"),
                    "%s response: success=%d, message='%s'",
                    service_name.c_str(),
                    response->success,
                    response->message.c_str());
        return response->success;
    }

    RCLCPP_ERROR(rclcpp::get_logger("spot_moveit"),
                 "Failed to call service '%s'", service_name.c_str());
    return false;
}

bool openGripper(){
    return callTriggerService("/spot_manipulation_driver/open_gripper");
}

bool closeGripper(){
    return callTriggerService("/spot_manipulation_driver/close_gripper");
}

bool unstowArm(){
    return callTriggerService("/spot_manipulation_driver/mini_unstow_arm");
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

    // -- go to mini unstow pose, moveit can plan from stow due to collisions
    unstowArm();
    if (!unstowArm()) {
    RCLCPP_WARN(node->get_logger(), "Arm failed to unstow.");
    }

    // std::this_thread::sleep_for(std::chrono::seconds(3));
    // saving initial unstowArm pose 
    geometry_msgs::msg::Pose carry_pose = move_group.getCurrentPose().pose;

    // --- subscribe to nav goal pose ---
    geometry_msgs::msg::Pose target_pose; // might need to be PoseStamped for sub
    // auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "/target_pose_topic", 10,
    //     [&target_pose](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    //     {target_pose = *msg;}
    //     );

    // creating fake target_pose coordinates for testing
    target_pose.position.x = 0.3; 
    target_pose.position.y = 0.0; 
    target_pose.position.z = 0.1;

    // change target pose orientation to be sideways for the gripper
    target_pose.orientation.x = 0.7221369743347168; 
    target_pose.orientation.y = -0.003546650754287839; 
    target_pose.orientation.z = 0.002715529641136527;
    target_pose.orientation.w = 0.6917356848716736;

    // --- define approach pose ---
    geometry_msgs::msg::Pose approach_pose = target_pose; // for real case
    // geometry_msgs::msg::Pose approach_pose = carry_pose; // for testing without nav goal topic
    approach_pose.position.x -= 0.09; // roughly 3.5 in

    move_group.setPoseTarget(approach_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to approach pose.");
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at approach pose.");
            return 1;
        }
 
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- define open hand pose, using open_gripper service call---
    openGripper();
    if (!openGripper()) {
    RCLCPP_WARN(node->get_logger(), "Gripper failed to open.");
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- approach ball, go to given target pose ---
    move_group.setPoseTarget(target_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to target pose.");
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at target pose.");
            return 1;
        }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- close hand --- 
    closeGripper();
    if (!closeGripper()) {
    RCLCPP_WARN(node->get_logger(), "Gripper failed to close.");
    }   

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- pick up ball and move arm up ---
    carry_pose.orientation = approach_pose.orientation;
    move_group.setPoseTarget(carry_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to carry pose.");
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at carry pose.");
            return 1;
        }

    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
