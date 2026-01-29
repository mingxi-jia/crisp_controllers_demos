# Copyright (c) 2024
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    com_port = LaunchConfiguration("com_port")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ns = LaunchConfiguration("namespace")

    description_pkg_share = FindPackageShare("robotiq_description")

    default_model_path = PathJoinSubstitution(
        [description_pkg_share, "urdf", "robotiq_2f_85_gripper.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            default_model_path,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "com_port:=",
            com_port,
        ]
    )

    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    update_rate_config_file = PathJoinSubstitution(
        [description_pkg_share, "config", "robotiq_update_rate.yaml"]
    )

    # Use local config with wildcard namespace prefix (direct path for volume mount)
    controllers_file = "/home/ros/ros2_ws/src/crisp_controllers_demos/crisp_controllers_robot_demos/config/robotiq_controllers.yaml"

    # Namespaced control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            robot_description_param,
            update_rate_config_file,
            controllers_file,
        ],
        remappings=[
            ("~/robot_description", "robot_description"),
        ],
    )

    # Namespaced robot state publisher with remapped joint_states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[robot_description_param],
        remappings=[
            ("joint_states", "gripper_joint_states"),
        ],
    )

    # Spawners pointing to the namespaced controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/gripper/controller_manager",
        ],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_gripper_controller",
            "-c",
            "/gripper/controller_manager",
        ],
    )

    robotiq_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_activation_controller",
            "-c",
            "/gripper/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "com_port",
                default_value="/dev/ttyUSB0",
                description="Serial port for the robotiq gripper",
            ),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Use fake hardware for testing",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="gripper",
                description="Namespace for the robotiq gripper",
            ),
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            robotiq_gripper_controller_spawner,
            robotiq_activation_controller_spawner,
        ]
    )
