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


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));


    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread exec_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(LOGGER, "Running planning node...");

    const std::string planning_group = "arm";
    // using moveit::planning_interface::MoveGroupInterface;
    // MoveGroupInterface move_group(node, planning_group);

    // after creating and spinning the node (see below)
    // moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    // move_group.waitForServers();

    moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm", "robot_description", "/spot_moveit");
    moveit::planning_interface::MoveGroupInterface move_group(node, move_group_options);
    RCLCPP_INFO(LOGGER, "Move group interface initialized...");


    // auto get_pose_service = node->create_service<std_srvs::srv::Trigger>(
    // "get_current_pose",
    // [&move_group](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
    //               std_srvs::srv::Trigger::Response::SharedPtr res) {
    //     geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    //     const auto &p = current_pose.pose.position;
    //     const auto &o = current_pose.pose.orientation;

    //     std::stringstream ss;
    //     ss << "Position: [x=" << p.x << ", y=" << p.y << ", z=" << p.z << "]\n"
    //        << "Orientation: [x=" << o.x << ", y=" << o.y << ", z=" << o.z << ", w=" << o.w << "]";

    //     res->success = true;
    //     res->message = ss.str();

    //     RCLCPP_INFO(LOGGER, "Current EE pose:\n%s", ss.str().c_str());
    // });


    
    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // --- define approach pose ---
    geometry_msgs::msg::Pose pick_pose;
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    pick_pose = current_pose;
    // pick_pose.position.x = -0.62; // 25.25 in. 
    // pick_pose.position.y = 0.0;
    pick_pose.position.z += 0.1;
    // pick_pose.orientation.x = -0.0; 
    // pick_pose.orientation.y = 1.0; 
    // pick_pose.orientation.z = -0.0;
    // pick_pose.orientation.w = 0.0;

    move_group.setPoseTarget(pick_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {

        // std::string dummy;
        // RCLCPP_INFO(LOGGER, "Press Enter to continue executing");
        // std::getline(std::cin, dummy);
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to pick pose.");
    
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at pick pose.");
            return 1;
        }
 
    // --- define open hand pose ---
    

    // --- approach ball, define pick pose ---


    // --- close hand ---


    // --- pick up ball and move arm up ---



    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
