from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from moveit_configs_utils import MoveItConfigsBuilder
from spot_description.get_accessories import get_accessories_from_env    




def generate_launch_description():

    # Load MoveIt configs using MoveItConfigsBuilder
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="spot", package_name="spot_moveit_config")
    #     .robot_description(file_path="config/spot.urdf.xacro")
    #     .robot_description_semantic(file_path="config/spot.srdf.xacro")
    #     .to_moveit_configs()
    # )

    #from move_group.launch
    # Launch args
    launch_args = [
        DeclareLaunchArgument('kinematic_model',
                            description='The kinematic model to use for the Spot description',
                            choices=['none', 'body_assist', 'mobile_manipulation'],
                            default_value='none')
    ]

    xacro_args = get_accessories_from_env()
    # xacro_args['kinematic_model'] = LaunchConfiguration('kinematic_model')
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=xacro_args)
    moveit_config_builder.robot_description_semantic(mappings=xacro_args)
    moveit_config = moveit_config_builder.to_moveit_configs()

    return LaunchDescription([
        Node(
            package='spot_arm_control',
            executable='pick_ball',
            namespace='spot_moveit',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                # moveit_config.robot_description_kinematics,
            ],
        ),
    ])
    