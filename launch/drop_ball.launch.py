from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import SetRemap, SetParameter, PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from spot_description.get_accessories import get_accessories_from_env    
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():

    # Launch args
    launch_args = [
        DeclareLaunchArgument('kinematic_model',
                            description='The kinematic model to use for the Spot description',
                            choices=['none', 'body_assist', 'mobile_manipulation'],
                            default_value='none')
    ]

    xacro_args = get_accessories_from_env()
    xacro_args['kinematic_model'] = LaunchConfiguration('kinematic_model')
    moveit_config_builder = MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
    moveit_config_builder.robot_description(mappings=xacro_args)
    moveit_config_builder.robot_description_semantic(mappings=xacro_args)
    moveit_config = moveit_config_builder.to_moveit_configs()

    return LaunchDescription([
        *launch_args,
        GroupAction(
            actions=[
                SetRemap(src='/joint_states', dst='/spot_driver/joint_states'),
                # SetRemap(src='/spot_moveit/robot_description', dst='/spot_driver/robot_description'),
                # PushRosNamespace('spot_moveit'),
                # SetParameter(name="octomap_resolution", value=0.03),
                # SetParameter(name="octomap_frame", value="spot_nav/map"),
                Node(
                    package='spot_arm_control',
                    executable='drop_ball',
                    # namespace='spot_moveit',
                    output='screen',
                    # remappings=[('/joint_states', '/spot_driver/joint_states')],
                    parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                    ],
                )
            ]
        )
    ])








