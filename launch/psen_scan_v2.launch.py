'''Copyright (c) 2020-2021 Pilz GmbH & Co. KG

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.'''

# This launch file runs the PSEN safety scanner driver together with a robot model and preconfigured RViz.
# It also reads the XML config file and publishes the Pilz zones marker to be shown in RViz.

import math
import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare all arguments with defaults
    declared_arguments = [
        DeclareLaunchArgument("sensor_ip", default_value="192.168.0.10", description="IP-Address of Safety laser scanner"),
        DeclareLaunchArgument("nr_subscribers", default_value="0", description="Number of subscribers connected to master scanner"),
        DeclareLaunchArgument("tf_prefix", default_value="laser_1", description="Name of the scanner"),
        DeclareLaunchArgument("tf_prefix_sub0", default_value="laser_1_subscriber0"),
        DeclareLaunchArgument("tf_prefix_sub1", default_value="laser_1_subscriber1"),
        DeclareLaunchArgument("tf_prefix_sub2", default_value="laser_1_subscriber2"),
        DeclareLaunchArgument("angle_start", default_value=str(-137.4 * math.pi / 180.0), description="Start angle of measurement in radian"),
        DeclareLaunchArgument("angle_end", default_value=str(137.4 * math.pi / 180.0), description="End angle of measurement in radian"),
        DeclareLaunchArgument("intensities", default_value="false", description="Publishing of intensities"),
        DeclareLaunchArgument("resolution", default_value=str(0.5 * math.pi / 180.0), description="Scan resolution in radian"),
        DeclareLaunchArgument("host_ip", default_value="auto", description="IP-Address of host machine"),
        DeclareLaunchArgument("host_udp_port_data", default_value="55115", description="UDP Port for monitoring frames"),
        DeclareLaunchArgument("host_udp_port_control", default_value="55116", description="UDP Port for commands"),
        DeclareLaunchArgument("fragmented_scans", default_value="false", description="Publish scan data per UDP packet"),
        DeclareLaunchArgument("config_file", default_value=os.path.join(os.getenv("HOME"), "PSENConfig/config.xml"), description="Path to scanner configuration file"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz")
    ]

    # Include bringup launch
    scanner_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('psen_scan_v2'),
                'launch',
                'bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'sensor_ip': LaunchConfiguration('sensor_ip'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'angle_start': LaunchConfiguration('angle_start'),
            'angle_end': LaunchConfiguration('angle_end'),
            'intensities': LaunchConfiguration('intensities'),
            'resolution': LaunchConfiguration('resolution'),
            'host_ip': LaunchConfiguration('host_ip'),
            'host_udp_port_data': LaunchConfiguration('host_udp_port_data'),
            'host_udp_port_control': LaunchConfiguration('host_udp_port_control'),
            'fragmented_scans': LaunchConfiguration('fragmented_scans'),
            'nr_subscribers': LaunchConfiguration('nr_subscribers'),
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        PathJoinSubstitution(
                            [FindPackageShare("psen_scan_v2"), "urdf", "safety_scanner.urdf.xacro"]
                        ),
                        " nr_scanners:=",
                        LaunchConfiguration("nr_subscribers"),
                        " prefix:=",
                        LaunchConfiguration("tf_prefix"),
                        " prefix_sub0:=",
                        LaunchConfiguration("tf_prefix_sub0"),
                        " prefix_sub1:=",
                        LaunchConfiguration("tf_prefix_sub1"),
                        " prefix_sub2:=",
                        LaunchConfiguration("tf_prefix_sub2"),
                    ]
                )
            }
        ],
    )

    # Config server node
    config_server = Node(
        package="psen_scan_v2",
        executable="config_server_node",
        name="config_server_node",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "frame_id": LaunchConfiguration("tf_prefix"),
            }
        ],
    )

    # Active zoneset node
    active_zoneset = Node(
        package="psen_scan_v2", executable="active_zoneset_node", name="active_zoneset_node"
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("psen_scan_v2"), "config", "config.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        declared_arguments
        + [scanner_include, robot_state_publisher, config_server, active_zoneset, rviz_node]
    )
