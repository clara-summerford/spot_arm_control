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

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("spot_moveit");

bool callTriggerService(const std::string &service_name)
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

geometry_msgs::msg::Pose rotatePoseRoll90(const geometry_msgs::msg::Pose &pose_body)
{
    tf2::Quaternion q_orig;
    tf2::fromMsg(pose_body.orientation, q_orig);

    // Local rotation: 90° about X-axis (roll)
    tf2::Quaternion q_rot;
    q_rot.setRPY(M_PI / 2.0, 0, 0);

    // Local = post-multiply
    tf2::Quaternion q_final = q_orig * q_rot;

    geometry_msgs::msg::Pose out = pose_body;
    out.orientation = tf2::toMsg(q_final);
    return out;
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


    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // std::thread exec_thread([&executor]() { executor.spin(); });

  

    const std::string planning_group = "arm";

    moveit::planning_interface::MoveGroupInterface::Options move_group_options("arm", "robot_description", "/spot_moveit");
    moveit::planning_interface::MoveGroupInterface move_group(node, move_group_options);
    // move_group.waitForServers();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);        // your main node (subscriber + TF)
    // executor.add_node(move_group.getNode()); // MoveIt node
    std::thread exec_thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(LOGGER, "Move group interface initialized...");
    std::string planning_frame = move_group.getPlanningFrame();
    RCLCPP_INFO(node->get_logger(), "MoveIt planning frame: %s", planning_frame.c_str());
    
    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // -- go to mini unstow pose, moveit can plan from stow due to collisions
    unstowArm();
    if (!unstowArm()) {
    RCLCPP_WARN(node->get_logger(), "Arm failed to unstow.");
    }

    // saving initial unstowArm pose 
    geometry_msgs::msg::Pose carry_pose = move_group.getCurrentPose().pose;

    // --- subscribe to nav goal pose ---
    // geometry_msgs::msg::PoseStamped raw_target_pose; // might need to be PoseStamped for sub
    // auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     "/ball_stable_pose", 10,
    //     [&raw_target_pose](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    //     {raw_target_pose = *msg;}
    //     );

    // RCLCPP_INFO(node->get_logger(), 
    //     "Target pose in frame (map): x=%.3f y=%.3f z=%.3f",
    //     raw_target_pose.pose.position.x,
    //     raw_target_pose.pose.position.y,
    //     raw_target_pose.pose.position.z);

    // --- subscribe to nav goal pose and transform ---
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    geometry_msgs::msg::PoseStamped raw_target_pose;
    geometry_msgs::msg::PoseStamped target_pose_body;

    bool got_pose = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ball_stable_pose", 10,
        [&node, &tf_buffer, &raw_target_pose, &target_pose_body, &got_pose](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            RCLCPP_INFO(node->get_logger(), "Subscriber callback triggered!");

            raw_target_pose = *msg;
            got_pose = true;

            RCLCPP_INFO(node->get_logger(),
                "Received ball pose in frame (%s): x=%.3f y=%.3f z=%.3f",
                msg->header.frame_id.c_str(),
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);

            try {
            // --- 1. TF map → body ---
            geometry_msgs::msg::TransformStamped tf_map_to_body =
                tf_buffer->lookupTransform(
                    "body",                
                    msg->header.frame_id, 
                    tf2::TimePointZero,
                    std::chrono::milliseconds(50));

            geometry_msgs::msg::PoseStamped pose_body;
            tf2::doTransform(*msg, pose_body, tf_map_to_body);

            RCLCPP_INFO(node->get_logger(),
                "Body frame (pre-rotation): x=%.3f y=%.3f z=%.3f",
                pose_body.pose.position.x,
                pose_body.pose.position.y,
                pose_body.pose.position.z);

            // --- 2. Rotate for grasp (90° roll in body frame) ---
            pose_body.pose = rotatePoseRoll90(pose_body.pose);

            target_pose_body = pose_body;

            RCLCPP_INFO(node->get_logger(),
                "Body frame (rotated): x=%.3f y=%.3f z=%.3f",
                target_pose_body.pose.position.x,
                target_pose_body.pose.position.y,
                target_pose_body.pose.position.z);

            } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(),
                "TF transform failed %s → body: %s",
                msg->header.frame_id.c_str(), ex.what());
        }
    });
    RCLCPP_INFO(node->get_logger(), "Waiting for /ball_stable_pose...");
    while (rclcpp::ok() && !got_pose) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node->get_logger(), "Got pose!");    

    // Wait until the first message arrives
    // rclcpp::Rate rate(10);  // 10 Hz
    // while (rclcpp::ok() && !got_pose) {
    //     rclcpp::spin_some(node);
    //     rate.sleep();
    // }
    


    // --- define approach pose ---
    // geometry_msgs::msg::Pose approach_pose = target_pose; // for real case
    // geometry_msgs::msg::Pose approach_pose = carry_pose; // for testing without nav goal topic

    // auto pose_utils = std::make_shared<PoseUtils>(node);
    // geometry_msgs::msg::PoseStamped target_in_map;
    // target_in_map.header.frame_id = "spot_nav/map";
    // target_in_map.pose = raw_target_pose.pose;

    // auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    // auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


    // offset farther back from ball and rotate hand
    // geometry_msgs::msg::PoseStamped target_in_body = pose_utils->transformMapToBody(target_in_map);
    target_pose_body.pose.position.x -= 0.1;

    RCLCPP_INFO(node->get_logger(), 
            "Target in planning frame (%s): x=%.3f y=%.3f z=%.3f",
            move_group.getPlanningFrame().c_str(),
            target_pose_body.pose.position.x,
            target_pose_body.pose.position.y,
            target_pose_body.pose.position.z);

    RCLCPP_INFO(node->get_logger(), 
            "Current pose in frame (%s): x=%.3f y=%.3f z=%.3f",
            move_group.getPlanningFrame().c_str(),
            carry_pose.position.x,
            carry_pose.position.y,
            carry_pose.position.z);

    // target_pose_body.pose = pose_utils->rotateHandRoll90(target_pose_body.pose);

    RCLCPP_INFO(node->get_logger(), 
            "Target in planning frame after rotation (%s): x=%.3f y=%.3f z=%.3f",
            move_group.getPlanningFrame().c_str(),
            target_pose_body.pose.position.x,
            target_pose_body.pose.position.y,
            target_pose_body.pose.position.z);

    move_group.setPoseTarget(target_pose_body);

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
    target_pose_body.pose.position.x += 0.1;
    move_group.setPoseTarget(target_pose_body.pose);
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

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- pick up ball and move arm up ---
    carry_pose.orientation = target_pose_body.pose.orientation;
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
