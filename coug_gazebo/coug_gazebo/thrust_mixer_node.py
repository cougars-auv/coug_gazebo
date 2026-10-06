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

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float64

# wamv_gazebo_dynamics_plugin.xacro
_WAMV_LINEAR_DRAG = 100.0  # xU
_WAMV_LINEAR_QUAD_DRAG = 150.0  # xUU
_WAMV_ANGULAR_DRAG = 800.0  # nR
_WAMV_ANGULAR_QUAD_DRAG = 800.0  # nRR

# wamv_gazebo_thruster_config.xacro/wamv_aft_thrusters.xacro
_WAMV_MAX_THRUST = 2353.5  # max_thrust_cmd
_WAMV_THRUSTER_Y = 1.027135  # engine position y (m)


class ThrustMixerNode(Node):
    def __init__(self) -> None:
        super().__init__("thrust_mixer_node")

        self.declare_parameter("input_topic", "cmd_vel_out")
        self.declare_parameter("left_output_topic", "thrusters/left/thrust")
        self.declare_parameter("right_output_topic", "thrusters/right/thrust")

        input_topic = self.get_parameter("input_topic").value
        left_output_topic = self.get_parameter("left_output_topic").value
        right_output_topic = self.get_parameter("right_output_topic").value

        self._input_sub = self.create_subscription(
            TwistStamped,
            input_topic,
            self._twist_callback,
            qos_profile_system_default,
        )
        self._left_pub = self.create_publisher(
            Float64, left_output_topic, qos_profile_system_default
        )
        self._right_pub = self.create_publisher(
            Float64, right_output_topic, qos_profile_system_default
        )

        self.get_logger().info("Initialization complete.")

    def _twist_callback(self, msg: TwistStamped) -> None:
        left_thrust_msg, right_thrust_msg = self._convert_to_thrust(msg)
        self._left_pub.publish(left_thrust_msg)
        self._right_pub.publish(right_thrust_msg)

    def _convert_to_thrust(self, msg: TwistStamped) -> tuple[Float64, Float64]:
        fwd = msg.twist.linear.x * (
            _WAMV_LINEAR_DRAG + _WAMV_LINEAR_QUAD_DRAG * abs(msg.twist.linear.x)
        )
        yaw = msg.twist.angular.z * (
            _WAMV_ANGULAR_DRAG + _WAMV_ANGULAR_QUAD_DRAG * abs(msg.twist.angular.z)
        )

        cmd_left = fwd / 2.0 - yaw / (2.0 * _WAMV_THRUSTER_Y)
        cmd_right = fwd / 2.0 + yaw / (2.0 * _WAMV_THRUSTER_Y)

        max_req = max(abs(cmd_left), abs(cmd_right))
        if max_req > _WAMV_MAX_THRUST:
            scale_factor = _WAMV_MAX_THRUST / max_req
            cmd_left *= scale_factor
            cmd_right *= scale_factor

        left_thrust_msg = Float64()
        left_thrust_msg.data = cmd_left

        right_thrust_msg = Float64()
        right_thrust_msg.data = cmd_right

        return left_thrust_msg, right_thrust_msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    thrust_mixer_node = ThrustMixerNode()
    try:
        rclpy.spin(thrust_mixer_node)
    except KeyboardInterrupt:
        pass
    finally:
        thrust_mixer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
