from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

    # Load MoveIt configs using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder(robot_name="spot", package_name="spot_moveit_config")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='spot_arm_control',
            executable='pick_ball',
            namespace='spot_moveit',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                # moveit_config.robot_description_semantic,
                # moveit_config.robot_description_kinematics,
            ],
        ),
    ])
    