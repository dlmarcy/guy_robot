# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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

#from launch import LaunchDescription
#from ament_index_python.packages import get_package_share_directory
#import launch_ros.actions
#import os
#import yaml
#from launch.substitutions import EnvironmentVariable
#import pathlib
#import launch.actions
#from launch.actions import DeclareLaunchArgument

#def generate_launch_description():
#	#ekf_params = os.path.join(get_package_share_directory('robot_base'), 'params', 'ekf_params.yaml')
#
#	return LaunchDescription([
#		launch_ros.actions.Node(
#		package='usb_cam',
#		executable='usb_cam_node_exe',
#		name='usb_cam_node',
#		output='screen',
#		#parameters=[ekf_params],
#	),
#])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	usb_cam_launch = os.path.join(get_package_share_directory('usb_cam'), 'launch', 'camera.launch.py')
	
	usb_cam_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(usb_cam_launch))

	return LaunchDescription([
		usb_cam_include,
	])

