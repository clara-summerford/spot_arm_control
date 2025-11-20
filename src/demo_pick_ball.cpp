#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("spot_moveit");

class SpotManipulator
{
public:
    SpotManipulator(rclcpp::Node::SharedPtr node)
        : node_(node), got_pose_(false)
    {
        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize MoveIt
        moveit::planning_interface::MoveGroupInterface::Options move_group_options(
            "arm", "robot_description", "/spot_moveit");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, move_group_options);

        // Configure MoveIt
        move_group_->setPlanningTime(5.0);
        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);

        RCLCPP_INFO(LOGGER, "MoveIt planning frame: %s",
                    move_group_->getPlanningFrame().c_str());

        // Subscribe to ball pose
        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ball_stable_pose", 10,
            std::bind(&SpotManipulator::poseCallback, this, std::placeholders::_1));
    }

    bool callTriggerService(const std::string &service_name)
    {
        auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(LOGGER, "Service '%s' not available", service_name.c_str());
            return false;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);
        
        // Wait for the result with timeout (node is already spinning in background)
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status == std::future_status::ready)
        {
            auto response = future.get();
            RCLCPP_INFO(LOGGER, "%s response: success=%d, message='%s'",
                        service_name.c_str(), response->success, response->message.c_str());
            return response->success;
        }
        
        RCLCPP_ERROR(LOGGER, "Failed to call service '%s' (timeout)", service_name.c_str());
        return false;
    }

    bool openGripper()
    {
        return callTriggerService("/spot_manipulation_driver/open_gripper");
    }

    bool closeGripper()
    {
        return callTriggerService("/spot_manipulation_driver/close_gripper");
    }

    bool unstowArm()
    {
        return callTriggerService("/spot_manipulation_driver/mini_unstow_arm");
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

    bool waitForPose(double timeout_seconds = 10.0)
    {
        RCLCPP_INFO(LOGGER, "Waiting for pose ...");

        auto start_time = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                if (got_pose_) {
                    return true;
                }
            }

            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration<double>(elapsed).count() > timeout_seconds) {
                RCLCPP_WARN(LOGGER, "Timeout waiting for ball pose");
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    }

    geometry_msgs::msg::PoseStamped getTargetPose()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return target_pose_body_;
    }

    bool executePickAndPlace()
    {
        // // Step 1: Unstow arm
        // RCLCPP_INFO(LOGGER, "Step 1: Unstowing arm...");
        // if (!unstowArm()) {
        //     RCLCPP_ERROR(LOGGER, "Failed to unstow arm");
        //     return false;
        // }
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // Save carry pose
        geometry_msgs::msg::Pose carry_pose = move_group_->getCurrentPose().pose;
        RCLCPP_INFO(LOGGER, "Carry pose saved: x=%.3f y=%.3f z=%.3f",
                    carry_pose.position.x, carry_pose.position.y, carry_pose.position.z);

        // Step 2: Wait for target pose
        RCLCPP_INFO(LOGGER, "Step 2: Waiting for ball pose...");
        if (!waitForPose()) {
            RCLCPP_ERROR(LOGGER, "Failed to receive ball pose");
            return false;
        }

        geometry_msgs::msg::PoseStamped target_pose = getTargetPose();
        RCLCPP_INFO(LOGGER, "Target pose received: x=%.3f y=%.3f z=%.3f",
                    target_pose.pose.position.x,
                    target_pose.pose.position.y,
                    target_pose.pose.position.z);

        // Step 3: Move to approach pose (10cm back from target)
        RCLCPP_INFO(LOGGER, "Step 3: Moving to approach pose...");
        geometry_msgs::msg::Pose approach_pose = target_pose.pose;
        approach_pose.position.x -= 0.1;
        if (!moveToPose(approach_pose, "approach")) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Step 4: Open gripper
        RCLCPP_INFO(LOGGER, "Step 4: Opening gripper...");
        if (!openGripper()) {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Step 5: Move to grasp pose
        RCLCPP_INFO(LOGGER, "Step 5: Moving to grasp pose...");
        if (!moveToPose(target_pose.pose, "grasp")) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Step 6: Close gripper
        RCLCPP_INFO(LOGGER, "Step 6: Closing gripper...");
        if (!closeGripper()) {
            RCLCPP_ERROR(LOGGER, "Failed to close gripper");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Step 7: Return to carry pose
        RCLCPP_INFO(LOGGER, "Step 7: Moving to carry pose...");
        carry_pose.orientation = target_pose.pose.orientation;
        if (!moveToPose(carry_pose, "carry")) {
            return false;
        }

        RCLCPP_INFO(LOGGER, "Pick and place completed successfully!");
        return true;
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(LOGGER, "Received ball pose in frame (%s): x=%.3f y=%.3f z=%.3f",
                    msg->header.frame_id.c_str(),
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z);

        try {
            // Create a copy with current time (other timing fix)
            geometry_msgs::msg::PoseStamped msg_copy = *msg;
            msg_copy.header.stamp = node_->now();  // Use current time


            // Transform from map to body frame
            geometry_msgs::msg::TransformStamped tf_map_to_body =
                tf_buffer_->lookupTransform(
                    "body",
                    msg_copy.header.frame_id,
                    // rclcpp::Time(0),
                    msg_copy.header.stamp,
                    std::chrono::milliseconds(100));

            geometry_msgs::msg::PoseStamped pose_body;
            tf2::doTransform(msg_copy, pose_body, tf_map_to_body);

            RCLCPP_INFO(LOGGER, "Body frame (pre-rotation): x=%.3f y=%.3f z=%.3f",
                        pose_body.pose.position.x,
                        pose_body.pose.position.y,
                        pose_body.pose.position.z);

            // Rotate for grasp (90° roll in body frame)
            pose_body.pose = rotatePoseRoll90(pose_body.pose);

            // Thread-safe update
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                target_pose_body_ = pose_body;
                got_pose_ = true;
            }

            RCLCPP_INFO(LOGGER, "Body frame (rotated): x=%.3f y=%.3f z=%.3f",
                        pose_body.pose.position.x,
                        pose_body.pose.position.y,
                        pose_body.pose.position.z);

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(LOGGER, "TF transform failed %s → body: %s",
                        msg->header.frame_id.c_str(), ex.what());
        }
    }

    bool moveToPose(const geometry_msgs::msg::Pose &target_pose, const std::string &pose_name)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->setPoseTarget(target_pose);

        RCLCPP_INFO(LOGGER, "Planning to %s pose: x=%.3f y=%.3f z=%.3f",
                    pose_name.c_str(),
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z);

        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(LOGGER, "Plan successful, executing...");
            if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(LOGGER, "Successfully moved to %s pose", pose_name.c_str());
                return true;
            } else {
                RCLCPP_ERROR(LOGGER, "Execution failed for %s pose", pose_name.c_str());
                return false;
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Planning failed for %s pose", pose_name.c_str());
            return false;
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    // Thread-safe shared state
    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped target_pose_body_;
    bool got_pose_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "spot_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Spin in background thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Give executor time to start
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Create manipulator object
    auto manipulator = std::make_shared<SpotManipulator>(node);
    std::this_thread::sleep_for(std::chrono::seconds(3));


    RCLCPP_INFO(LOGGER, "Starting pick and place operation...");

    // Execute the pick and place sequence
    bool success = manipulator->executePickAndPlace();

    if (success) {
        RCLCPP_INFO(LOGGER, "Operation completed successfully");
    } else {
        RCLCPP_ERROR(LOGGER, "Operation failed");
    }

    // Cleanup
    executor.cancel();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }

    rclcpp::shutdown();
    return success ? 0 : 1;
}