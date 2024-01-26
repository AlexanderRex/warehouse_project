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

    rviz_config_file = get_file_path("path_planner_server", "config", "pathplanning.rviz")

    controller_yaml_real = get_file_path("path_planner_server", "config", "controller_real.yaml")
    bt_navigator_yaml_real = get_file_path("path_planner_server", "config", "bt_real.yaml")
    planner_yaml_real = get_file_path("path_planner_server", "config", "planner_server_real.yaml")
    recovery_yaml_real = get_file_path("path_planner_server", "config", "recovery_real.yaml")

    controller_yaml_sim = get_file_path("path_planner_server", "config", "controller_sim.yaml")
    bt_navigator_yaml_sim = get_file_path("path_planner_server", "config", "bt_sim.yaml")
    planner_yaml_sim = get_file_path("path_planner_server", "config", "planner_server_sim.yaml")
    recovery_yaml_sim = get_file_path("path_planner_server", "config", "recovery_sim.yaml")

    nav2_con_node_sim = Node(package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[controller_yaml_sim], remappings=[('/cmd_vel', '/robot/cmd_vel')],
            condition=IfCondition(use_sim_time))
    nav2_con_node_real = Node(package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[controller_yaml_real], remappings=[('/cmd_vel', '/robot/cmd_vel')],
            condition=UnlessCondition(use_sim_time))

    nav2_plan_node_sim = Node(package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[planner_yaml_sim],
            condition=IfCondition(use_sim_time))
    nav2_plan_node_real = Node(package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[planner_yaml_real],
            condition=UnlessCondition(use_sim_time))

    nav2_rec_sim = Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server',
            parameters=[recovery_yaml_sim], output='screen',
            condition=IfCondition(use_sim_time))
    nav2_rec_real = Node(package='nav2_recoveries', executable='recoveries_server', name='recoveries_server',
            parameters=[recovery_yaml_real], output='screen',
            condition=UnlessCondition(use_sim_time))

    nav2_bt_nav_node_sim = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[bt_navigator_yaml_sim],
            condition=IfCondition(use_sim_time))
    nav2_bt_nav_node_real = Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[bt_navigator_yaml_real],
            condition=UnlessCondition(use_sim_time))
    
    nav2_life_man = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_pathplanner',
            output='screen', parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file])

    return LaunchDescription(
        [declare_use_sim_time,
        rviz_node,
        nav2_con_node_sim, nav2_con_node_real,
        nav2_plan_node_sim, nav2_plan_node_real,
        nav2_rec_sim, nav2_rec_real,
        nav2_bt_nav_node_sim, nav2_bt_nav_node_real,
        nav2_life_man
        ]
    )