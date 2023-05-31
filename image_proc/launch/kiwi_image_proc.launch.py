# Copyright (c) 2020, Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# =============================================================================
"""
Code Information:
    Maintainer: Eng. Salomon Granada Ulloque
	Mail: salomon.granada@kiwibot.com
	Kiwibot / Ai & Robotics Team
"""
# =============================================================================

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory("vision_bringup"), "params", "vision_params.yaml"
    )

    composable_nodes = []
    # read params file yaml into dict
    params_file_dict = yaml.load(open(params_file), Loader=yaml.FullLoader)
    for param, value in params_file_dict["video_mapping_cpp"]["ros__parameters"][
        "image_undistort"
    ].items():

        # Skip if its not <cam_label>_enable param as is the one we need to check
        if not "_enable" in param:
            continue

        # Override param value if env var is set. This follows the Talos env var convention
        is_enable_env_var = os.getenv(
            f"VIDEO_MAPPING_CPP_IMAGE_UNDISTORT_{param.split('_')[0].upper()}_ENABLE"
        )
        launch_rect = value if is_enable_env_var == None else int(is_enable_env_var)

        if launch_rect:
            cam_to_rect = param.split("_")[0]
            composable_nodes.append(
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name=f"{cam_to_rect}_rectify_component",
                    namespace=f"{cam_to_rect}_rect",
                    # Remap subscribers and publishers
                    remappings=[
                        ("image", f"/video_mapping/{cam_to_rect}/image_raw"),
                        ("camera_info", f"/video_mapping/{cam_to_rect}/camera_info"),
                        ("image_rect", f"/video_mapping/{cam_to_rect}/image_rect"),
                    ],
                )
            )

    arg_container = DeclareLaunchArgument(
        name="container",
        default_value="",
        description=(
            "Name of an existing node container to load launched nodes into. "
            "If unset, a new container will be created."
        ),
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals("container", ""),
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals("container", ""),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("container"),
    )

    return LaunchDescription(
        [
            arg_container,
            image_processing_container,
            load_composable_nodes,
        ]
    )
