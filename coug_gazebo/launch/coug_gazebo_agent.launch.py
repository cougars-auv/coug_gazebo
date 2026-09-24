# Copyright 2026 BYU FROST Lab
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

import json
import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitution import Substitution
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def agent_frame(agent_ns: str | Substitution, frame: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, f"/{frame}' if '", agent_ns, f"' != '' else '{frame}'"])


def load_launch_params(path: str, top_key: str) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key]["coug_gazebo_agent_launch"]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")
    initial_position_str = LaunchConfiguration("initial_position").perform(context)
    initial_orientation_str = LaunchConfiguration("initial_orientation").perform(context)

    agent_ns_str = agent_ns.perform(context)

    position = json.loads(initial_position_str) if initial_position_str else [0.0, 0.0, 0.0]
    orientation = (
        json.loads(initial_orientation_str) if initial_orientation_str else [0.0, 0.0, 0.0]
    )

    config_dir = os.environ["CONFIG_DIR"]
    coug_description_dir = get_package_share_directory("coug_description")
    coug_gazebo_dir = get_package_share_directory("coug_gazebo")

    agent_bridge_config_file = os.path.join(coug_gazebo_dir, "config", "agent_bridge.yaml")
    agent_camera_bridge_config_file = os.path.join(
        coug_gazebo_dir, "config", "agent_camera_bridge.yaml"
    )
    thruster_bridge_config_file = os.path.join(coug_gazebo_dir, "config", "thruster_bridge.yaml")

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_gazebo_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = (
        LaunchConfiguration("scenario_param_file").perform(context) or agent_param_file
    )

    fleet_launch_params = load_launch_params(
        os.path.join(config_dir, "fleet", "coug_gazebo_params.yaml"), "/**"
    )
    agent_launch_params = load_launch_params(
        os.path.join(config_dir, f"{agent_ns_str}_params.yaml"), f"/{agent_ns_str}"
    )
    urdf_filename = agent_launch_params.get("urdf_file", fleet_launch_params.get("urdf_file"))
    urdf_file = os.path.join(coug_description_dir, "urdf", urdf_filename)

    thrust_actions: list[Action] = []
    if urdf_filename == "wamv.gazebo.xacro":
        thrust_actions.append(
            RosGzBridge(
                bridge_name="thruster_bridge_node",
                config_file=thruster_bridge_config_file,
                container_name="/gazebo_container",
                namespace=f"/{agent_ns_str}",
                use_composition=True,
                extra_bridge_params={
                    "use_sim_time": use_sim_time,
                    "expand_gz_topic_names": True,
                },
            )
        )
        for side, sign in (("left", "-"), ("right", "+")):
            thrust_actions.append(
                Node(
                    package="topic_tools",
                    executable="transform",
                    name=f"{side}_thrust_mixer_node",
                    arguments=[
                        f"/{agent_ns_str}/cmd_vel_out",
                        f"/{agent_ns_str}/thrusters/{side}/thrust",
                        "std_msgs/msg/Float64",
                        (
                            "(s := (100.0 * m.linear.x + 150.0 * m.linear.x * abs(m.linear.x))"
                            " / 2.0, "
                            "y := (800.0 * m.angular.z + 800.0 * m.angular.z * abs(m.angular.z))"
                            " / 2.05427, "
                            f"std_msgs.msg.Float64(data=(s {sign} y)"
                            " / max(1.0, (abs(s) + abs(y)) / 2353.5)))[-1]"
                        ),
                        "--field",
                        "twist",
                        "--import",
                        "std_msgs",
                        "--wait-for-start",
                        "--qos-reliability",
                        "reliable",
                    ],
                )
            )

    return [
        *thrust_actions,
        Node(
            package="ros_gz_sim",
            executable="create",
            name="create",
            arguments=[
                "-name",
                agent_ns_str,
                "-string",
                Command(["xacro ", urdf_file, " agent_ns:=", agent_ns_str]),
                "-x",
                str(position[0]),
                "-y",
                str(position[1]),
                "-z",
                str(position[2]),
                "-R",
                str(orientation[0]),
                "-P",
                str(orientation[1]),
                "-Y",
                str(orientation[2]),
            ],
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        RosGzBridge(
            bridge_name="agent_bridge_node",
            config_file=agent_bridge_config_file,
            container_name="/gazebo_container",
            namespace=f"/{agent_ns_str}",
            use_composition=True,
            extra_bridge_params={
                "use_sim_time": use_sim_time,
                "expand_gz_topic_names": True,
            },
        ),
        RosGzBridge(
            bridge_name="agent_camera_bridge_node",
            config_file=agent_camera_bridge_config_file,
            container_name="/gazebo_container",
            namespace=f"/{agent_ns_str}",
            use_composition=True,
            extra_bridge_params={
                "use_sim_time": use_sim_time,
                "expand_gz_topic_names": True,
                "override_frame_id": agent_frame(agent_ns, "depth_camera_optical_link"),
            },
        ),
        Node(
            package="topic_tools",
            executable="relay",
            name="rgb_camera_info_relay_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="coug_gazebo",
            executable="imu_covariance",
            name="imu_covariance_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="coug_gazebo",
            executable="mag_covariance",
            name="mag_covariance_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="coug_gazebo",
            executable="navsat_covariance",
            name="navsat_covariance_node",
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "agent_ns",
                default_value="rover1gz",
            ),
            DeclareLaunchArgument(
                "scenario_param_file",
                default_value="",
            ),
            DeclareLaunchArgument(
                "initial_position",
                default_value="",
            ),
            DeclareLaunchArgument(
                "initial_orientation",
                default_value="",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
