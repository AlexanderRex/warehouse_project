from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def get_file_path(package, subdir, filename):
    return os.path.join(get_package_share_directory(package), subdir, filename)

def generate_launch_description():
    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                 description='True: Use Simulation Clock & False: Use Real Robot Clock')
    # use launch configuration in nodes
    use_sim_time = LaunchConfiguration('use_sim_time')   # <--- you can use this use_sim_time into your node definitions

    # get config files
    amcl_sim = get_file_path("localization_server", "config", "amcl_config_sim.yaml")
    amcl_real = get_file_path("localization_server", "config", "amcl_config_real.yaml")
    map_sim = get_file_path("map_server", "config", "warehouse_map_sim.yaml")
    map_real = get_file_path("map_server", "config", "warehouse_map_real.yaml")

    # declare nodes
    map_sim_node = Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                        parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_sim}],
                        condition=IfCondition(use_sim_time))
    map_real_node = Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                        parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_real}],
                        condition=UnlessCondition(use_sim_time))

    amcl_sim_node = Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                        parameters=[amcl_sim], condition=IfCondition(use_sim_time))
    amcl_real_node = Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                        parameters=[amcl_real], condition=UnlessCondition(use_sim_time))

    lifecycle_node = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", 
                          name="lifecycle_manager_localization", output="screen",
                          parameters=[{"use_sim_time": use_sim_time}, {"autostart": True},
                                      {"node_names": ["map_server", "amcl"]}])


    return LaunchDescription(
        [declare_use_sim_time,
         map_sim_node, map_real_node,
         amcl_sim_node, amcl_real_node,
         lifecycle_node
        ]
    )
