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
import shutil
from typing import Any
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.logging import launch_config
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


def is_agent(agent_ns: LaunchConfiguration, *names: str) -> PythonExpression:
    return PythonExpression(["'", agent_ns, "' in ", str(names)])


def load_launch_params(
    path: str, top_key: str, launch_key: str = "coug_gazebo_agent_launch"
) -> dict[str, Any]:
    try:
        with open(path) as config_file:
            config = yaml.safe_load(config_file)
        params = config[top_key][launch_key]["ros__parameters"]
        return dict(params)
    except (KeyError, TypeError, OSError):
        return {}


def launch_setup(context: LaunchContext, *args: Any, **kwargs: Any) -> list[Action]:
    use_sim_time = LaunchConfiguration("use_sim_time")
    agent_ns = LaunchConfiguration("agent_ns")

    agent_ns_str = agent_ns.perform(context)
    initial_position_str = LaunchConfiguration("initial_position").perform(context)
    initial_orientation_str = LaunchConfiguration("initial_orientation").perform(context)
    scenario_param_path = LaunchConfiguration("scenario_param_file").perform(context)

    position = json.loads(initial_position_str) if initial_position_str else [0.0, 0.0, 0.0]
    orientation = (
        json.loads(initial_orientation_str) if initial_orientation_str else [0.0, 0.0, 0.0]
    )

    config_dir = os.environ["CONFIG_DIR"]
    coug_gazebo_dir = get_package_share_directory("coug_gazebo")

    agent_bridge_config_file = os.path.join(coug_gazebo_dir, "config", "agent_bridge.yaml")
    agent_camera_bridge_config_file = os.path.join(
        coug_gazebo_dir, "config", "agent_camera_bridge.yaml"
    )

    fleet_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), "fleet", "coug_gazebo_params.yaml"]
    )
    agent_param_file = PathJoinSubstitution(
        [EnvironmentVariable("CONFIG_DIR"), [agent_ns, "_params.yaml"]]
    )
    scenario_param_file = scenario_param_path or agent_param_file

    fleet_param_path = os.path.join(config_dir, "fleet", "coug_gazebo_params.yaml")
    agent_param_path = os.path.join(config_dir, f"{agent_ns_str}_params.yaml")

    launch_params = {
        **load_launch_params(fleet_param_path, "/**"),
        **load_launch_params(agent_param_path, f"/{agent_ns_str}"),
        **load_launch_params(scenario_param_path, "/**"),
        **load_launch_params(scenario_param_path, f"/{agent_ns_str}"),
    }
    urdf_filename = launch_params["urdf_file"]
    urdf_file = os.path.join(coug_gazebo_dir, "urdf", urdf_filename)

    world_launch_params = {
        **load_launch_params(fleet_param_path, "/**", "coug_gazebo_world_launch"),
        **load_launch_params(scenario_param_path, "/**", "coug_gazebo_world_launch"),
    }
    world_filename = world_launch_params["world_file"]
    world_file = os.path.join(coug_gazebo_dir, "worlds", world_filename)
    world_origin = ElementTree.parse(world_file).find("world/spherical_coordinates")
    if world_origin is None:
        raise RuntimeError(f"No 'spherical_coordinates' set in '{world_file}'.")

    ardupilot_condition = IfCondition(is_agent(agent_ns, "yboat1gz"))
    ardupilot_home = ",".join(
        [
            world_origin.findtext("latitude_deg", "0").strip(),
            world_origin.findtext("longitude_deg", "0").strip(),
            world_origin.findtext("elevation", "0").strip(),
            "0",
        ]
    )
    ardupilot_instance = str(launch_params["ardupilot_instance"])
    ardupilot_param_filename = launch_params["ardupilot_param_file"]
    ardupilot_param_file = os.path.join(coug_gazebo_dir, "ardupilot", ardupilot_param_filename)

    ardupilot_dir = os.path.join(launch_config.log_dir, "ardupilot", agent_ns_str)
    if ardupilot_condition.evaluate(context):
        shutil.copytree(
            os.path.join(coug_gazebo_dir, "lua"),
            os.path.join(ardupilot_dir, "scripts"),
            dirs_exist_ok=True,
        )

    return [
        Node(
            package="coug_gazebo",
            executable="imu_covariance",
            name="imu_covariance_node",
            condition=UnlessCondition(is_agent(agent_ns, "yboat1gz")),
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
            condition=UnlessCondition(is_agent(agent_ns, "yboat1gz")),
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
            condition=UnlessCondition(is_agent(agent_ns, "yboat1gz")),
            parameters=[
                fleet_param_file,
                agent_param_file,
                scenario_param_file,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="coug_gazebo",
            executable="thrust_mixer",
            name="thrust_mixer_node",
            condition=IfCondition(is_agent(agent_ns, "wamv1gz")),
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
            package="ros_gz_sim",
            executable="create",
            name="create",
            arguments=[
                "-name",
                agent_ns_str,
                "-string",
                Command(
                    [
                        "xacro ",
                        urdf_file,
                        " agent_ns:=",
                        agent_ns_str,
                        " ardupilot_instance:=",
                        ardupilot_instance,
                    ]
                ),
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
        ExecuteProcess(
            cmd=[
                "ardurover",
                "--wipe",
                "--model",
                "JSON",
                "-I",
                ardupilot_instance,
                "--home",
                ardupilot_home,
                "--defaults",
                ardupilot_param_file,
            ],
            cwd=ardupilot_dir,
            condition=ardupilot_condition,
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
                default_value="auv0",
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
