# Copyright (c) Contributors to the Open 3D Engine Project.
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

from simulation_interfaces.srv import SpawnEntity
from simulation_interfaces.msg import Result

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node


class RobotSpawner(Node):

    def __init__(self):
        super().__init__('robot_spawner_client')

        self.declare_parameter('robot_uri', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('robot_initial_position', [0.0, 0.0, 0.0])
        self.declare_parameter('robot_initial_orientation', [0.0, 0.0, 0.0, 1.0])

        try:
            self.robot_uri = self.get_parameter('robot_uri').get_parameter_value().string_value
            self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        except ParameterUninitializedException:
            self.get_logger().warn("Nothing to spawn. 'robot_uri' or 'robot_name' parameter not set.")
            return

        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.robot_initial_position = self.get_parameter('robot_initial_position').get_parameter_value().double_array_value
        self.robot_initial_orientation = self.get_parameter('robot_initial_orientation').get_parameter_value().double_array_value

        if len(self.robot_initial_position) != 3:
            self.get_logger().error("robot_initial_position must have 3 values")
            return

        if len(self.robot_initial_orientation) != 4:
            self.get_logger().error("robot_initial_orientation must have 4 values (quaternion)")
            return

        self._spawn_client = self.create_client(SpawnEntity, 'spawn_entity')

        while not self._spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawning service not available, waiting...')

        spawn_result = self._call_spawn_service()
        if spawn_result is None:
            self.get_logger().error("Service call failed.")
            return

        if spawn_result.result.result == Result.RESULT_OK:
            self.get_logger().info(f'{self.robot_namespace} spawned as {spawn_result.entity_name}.')
        else:
            self.get_logger().error(f'Failed to spawn {self.robot_namespace}: {spawn_result.result.error_message}')

    def _call_spawn_service(self):
        self.get_logger().info(f"Spawning robot: {self.robot_namespace}/{self.robot_name} on ("
                               f"{self.robot_initial_position[0]}, "
                               f"{self.robot_initial_position[1]}, "
                               f"{self.robot_initial_position[2]}) with orientation ("
                               f"{self.robot_initial_orientation[0]}, "
                               f"{self.robot_initial_orientation[1]}, "
                               f"{self.robot_initial_orientation[2]}, "
                               f"{self.robot_initial_orientation[3]})")

        request = SpawnEntity.Request()
        request.uri = self.robot_uri
        request.name = self.robot_name
        request.entity_namespace = self.robot_namespace
        request.allow_renaming = True
        request.initial_pose.pose.position.x = self.robot_initial_position[0]
        request.initial_pose.pose.position.y = self.robot_initial_position[1]
        request.initial_pose.pose.position.z = self.robot_initial_position[2]
        request.initial_pose.pose.orientation.x = self.robot_initial_orientation[0]
        request.initial_pose.pose.orientation.y = self.robot_initial_orientation[1]
        request.initial_pose.pose.orientation.z = self.robot_initial_orientation[2]
        request.initial_pose.pose.orientation.w = self.robot_initial_orientation[3]

        response_future = self._spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, response_future)

        return response_future.result()


def main(args=None):
    rclpy.init(args=args)

    robot_spawner = RobotSpawner()
    robot_spawner.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
