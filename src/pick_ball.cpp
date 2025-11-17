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

/// Normalize a quaternion in-place
/// Probably not too useful but its been getting me good results

std::atomic<double> current_force_z = 0.0;

void normalizeQuaternion(geometry_msgs::msg::Pose &pose)
{
    double n = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);
    if (n < 1e-12) {
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
    } else {
        pose.orientation.x /= n;
        pose.orientation.y /= n;
        pose.orientation.z /= n;
        pose.orientation.w /= n;
    }
}

// --- compute Cartesian path and execute ---
// double computeAndExecuteCartesianPath(
//     moveit::planning_interface::MoveGroupInterface &move_group,
//     const std::vector<geometry_msgs::msg::Pose> &waypoints,
//     moveit::planning_interface::MoveGroupInterface::Plan &plan,
//     double eef_step = 0.001,
//     double jump_threshold = 0.0,
//     bool avoid_collisions = true,
//     double min_success_fraction = 0.5)
// {
//     moveit_msgs::msg::RobotTrajectory trajectory;
//     double fraction = move_group.computeCartesianPath(
//         waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);

//     RCLCPP_INFO(LOGGER, "computeCartesianPath fraction: %.2f%%", fraction * 100.0);

//     if (fraction >= min_success_fraction) {
//         plan.trajectory = trajectory;
//         move_group.execute(plan);
//         RCLCPP_INFO(LOGGER, "Executed Cartesian trajectory.");
//     } else {
//         RCLCPP_WARN(LOGGER, "Cartesian path below threshold: %.2f%%", fraction * 100.0);
//     }
//     return fraction;
// }


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // --- Fetch robot_description from MoveGroup node ---
    // auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    //     node, "/spot_moveit/move_group" // this is the name of the NODE with the robot_description parameter
    // );

    // Wait until MoveGroup node is up and publishing parameters
    // while (!param_client->wait_for_service(1)) {
    //     RCLCPP_INFO(node->get_logger(), "Waiting for /spot_moveit/move_group parameters...");
    // }

    // Get the URDF string
    // std::string urdf_string;
    // try {
    //     urdf_string = param_client->get_parameter<std::string>("robot_description");
    //     RCLCPP_INFO(node->get_logger(), "Successfully fetched robot_description");
    // } catch (const std::exception &e) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to get robot_description: %s", e.what());
    //     return 1;
    // }

    // // --- Initialize RobotModel
    // auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
    //     node,
    //     urdf_string,
    // );
    // moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    const std::string planning_group = "arm";
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, planning_group);

    RCLCPP_INFO(LOGGER, "Press Enter to continue executing");


    auto get_pose_service = node->create_service<std_srvs::srv::Trigger>(
    "get_current_pose",
    [&move_group](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                  std_srvs::srv::Trigger::Response::SharedPtr res) {
        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        const auto &p = current_pose.pose.position;
        const auto &o = current_pose.pose.orientation;

        std::stringstream ss;
        ss << "Position: [x=" << p.x << ", y=" << p.y << ", z=" << p.z << "]\n"
           << "Orientation: [x=" << o.x << ", y=" << o.y << ", z=" << o.z << ", w=" << o.w << "]";

        res->success = true;
        res->message = ss.str();

        RCLCPP_INFO(LOGGER, "Current EE pose:\n%s", ss.str().c_str());
    });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread exec_thread([&executor]() { executor.spin(); });
    
    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // --- including bowl for visualization purposes, no collision
    // moveit_msgs::msg::CollisionObject object;
    // object.id = "target_sphere";
    // object.header.frame_id = "world"; // keep this at world!!
    // object.primitives.resize(1);
    // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    // object.primitives[0].dimensions = 0.03; // estimate ball radius of 3cm

    // geometry_msgs::msg::Pose obj_pose;
    // obj_pose.position.x = -0.61;
    // obj_pose.position.y = 0.0;
    // obj_pose.position.z = -0.03; //center of sphere, will be height of base of bowl
    // obj_pose.orientation.w = 1.0;
    // object.pose = obj_pose;
    // moveit::planning_interface::PlanningSceneInterface psi;
    // psi.applyCollisionObject(object);

    // --- Define pick pose ---
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

    normalizeQuaternion(pick_pose);
    move_group.setPoseTarget(pick_pose);
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {

        std::string dummy;
        RCLCPP_INFO(LOGGER, "Press Enter to continue executing");
        std::getline(std::cin, dummy);
        // move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "Moved to pick pose.");
    
        } else {
            RCLCPP_WARN(LOGGER, "Planning failed at pick pose.");
            return 1;
        }
 

    rclcpp::shutdown();
    exec_thread.join();
    return 0;
}
