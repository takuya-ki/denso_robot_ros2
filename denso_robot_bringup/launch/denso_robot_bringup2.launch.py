# Copyright (c) 2021 DENSO WAVE INCORPORATED
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: DENSO WAVE INCORPORATED

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import OrSubstitution
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.actions import ExecuteProcess

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    ip_address = LaunchConfiguration('ip_address')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    namespace = LaunchConfiguration('namespace')
    launch_rviz = LaunchConfiguration('launch_rviz')
    sim = LaunchConfiguration('sim')
    with_tool = LaunchConfiguration('with_tool')
    verbose = LaunchConfiguration('verbose')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controller = LaunchConfiguration('robot_controller')
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')

    denso_robot_core_pkg = get_package_share_directory('denso_robot_core')

    denso_robot_control_parameters = {
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_config_file': PathJoinSubstitution([denso_robot_core_pkg, 'config', 'config.xml'])}

    moveit_config = (
        MoveItConfigsBuilder("denso_robot", robot_description="robot_description",
                             package_name="denso_robot_moveit_config")
        .robot_description(file_path=get_package_share_directory(str(description_package.perform(context))) + "/urdf/" + str(description_file.perform(context)),
                           mappings={
                            "ip_address": str(ip_address.perform(context)),
                            "model": str(denso_robot_model.perform(context)),
                            "send_format": str(send_format.perform(context)),
                            "recv_format": str(recv_format.perform(context)),
                            "namespace": str(namespace.perform(context)),
                            "verbose": str(verbose.perform(context)),
                            "sim": str(sim.perform(context)),
                            "with_tool": str(with_tool.perform(context)),
                            "ros2_control_hardware_type": str(ros2_control_hardware_type.perform(context)),
                            },
                            )
        .robot_description_semantic(file_path="srdf/denso_robot.srdf.xacro",
                           mappings={
                            "model": "cobotta",
                            "namespace": str(namespace.perform(context)),
                            },
                            )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .trajectory_execution(file_path="robots/cobotta/config/moveit_controllers.yaml")
        .joint_limits(file_path="robots/cobotta/config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        #.pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            #pipelines=["ompl", "pilz_industrial_motion_planner"],
            #default_planning_pipeline="pilz_industrial_motion_planner",
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        # .planning_scene_monitor()
        # .sensors_3d()
        .to_moveit_configs()
    )

    # 今のaptのバージョンだと以下のように追加で設定する必要がある
    # https://github.com/ros-planning/moveit2/pull/2270
    # 上記がマージされたバージョンがaptで入ると
    # denso_robot_moveit_config/config/pilz_industrial_motion_planner_planning.yaml
    # での設定が正常に読み込まれるはずなので下記は不要。
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    # Start the actual move_group node/action server
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,    # colored log
        parameters=[moveit_config.to_dict(),
                    move_group_capabilities,],
    )

# --------- Robot Control Node (only if 'sim:=false') ---------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config', controllers_file
        ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(OrSubstitution(sim, PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
        parameters=[
            moveit_config.robot_description,
            robot_controllers,
            denso_robot_control_parameters
        ],
        output="both",
    )
# -------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['denso_joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '-c', '/controller_manager'])

# --------- rviz with moveit configuration ---------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), 'rviz', 'view_robot.rviz'])

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ])
    # Static TF
    default_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world' 'base_link'],
        condition=LaunchConfigurationEquals('description_package', 'denso_robot_descriptions')
        )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

# --------- Gazebo Nodes (only if 'sim:=true') ---------
    set_param_use_sim_time = SetParameter(
            name='use_sim_time', value=True,
            condition=IfCondition(OrSubstitution(sim, PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))))

    # Sets Paths for ignition#
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(get_package_share_directory('denso_robot_descriptions'))
           + ':' + os.path.dirname(get_package_share_directory(str(LaunchConfiguration('description_package').perform(context))))
    }

    ign_gazebo = ExecuteProcess(
            condition=IfCondition(OrSubstitution(sim, PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
            cmd=['ign gazebo -r', 'empty.sdf'],
            output='screen',
            additional_env=env,
            shell=True
    )

    ignition_spawn_entity_node = Node(
        condition=IfCondition(OrSubstitution(sim, PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'cobotta',
                   '-allow_renaming', 'true'],
    )

    bridge = Node(
        condition=IfCondition(OrSubstitution(sim, PythonExpression(["'", ros2_control_hardware_type, "'=='ign_sim'"]))),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
# -------------------

    nodes_to_start = [
        set_param_use_sim_time,
        rviz_node,
        default_static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        ign_gazebo,
        ignition_spawn_entity_node,
        bridge,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

# Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            description='Type/series of used denso robot.'))
    # TODO: shall we let the user to only select from a list of robots ??
    # choices=['cobotta', 'vs060', 'vs087']))
    declared_arguments.append(
        DeclareLaunchArgument(
            'send_format', default_value='288',
            description='Data format for sending commands to the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'recv_format', default_value='292',
            description='Data format for receiving robot status.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'bcap_slave_control_cycle_msec', default_value='8.0',
            description='Control frequency.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'))
# Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package', default_value='denso_robot_descriptions',
            description='Description package with robot URDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom description.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file', default_value='denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package', default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom moveit config.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file', default_value='denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description="Prefix of the joint names, useful for" \
                + " multi-robot setup. If changed than also joint names in the controllers'" \
                + " configuration have to be updated."))
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file', default_value='denso_robot_controllers.yaml',
            description='YAML file with the controllers configuration.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller', default_value='denso_joint_trajectory_controller',
            description='Robot controller to start.'))
# Execution arguments (Rviz and Gazebo)
    declared_arguments.append(
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz?'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='This parameter is deprecated. Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Print out additional debug information.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_planner', default_value='ompl',
            description='Use "ompl" or "pilz"'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'with_tool', default_value='true',
            description='Whether there is a tool or not.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'ros2_control_hardware_type', default_value='',
            description='Specifies the hardware type to configure.[[ign_sim, mock_components]]'))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
