# Robotic Navigation and Mapping Package

This package provides a comprehensive suite of tools for robotic navigation and mapping, integrating functionalities for SLAM (Simultaneous Localization and Mapping), localization, map serving, and path planning. It is designed to be compatible with ROS-based robotic systems.

# Components
The package includes the following main components:

Cartographer SLAM: This component is dedicated to implementing SLAM (Simultaneous Localization and Mapping) capabilities using the Cartographer library. It includes configuration files for setting up Cartographer, ROS launch files for initiating the SLAM process, and a suite of unit tests to ensure the quality and standard compliance of the code.

Localization Server: This part of the package provides localization services using AMCL (Adaptive Monte Carlo Localization). It contains configuration files tailored for both real and simulated environments, launch files to start the localization node within a ROS framework, and tests to maintain code integrity and standards.

Map Server: The Map Server serves as a vital component for navigation, handling the storage and serving of map data. It includes configuration files along with map files in .pgm and .yaml formats, launch files for operating the map server, and a comprehensive test suite to ensure the functionality of the map server component.

Path Planner Server: This module is responsible for path planning, integrating various strategies for efficient and safe navigation. It encompasses configuration files for different environments and scenarios, launch files for deploying path planning services, and a set of tests dedicated to verifying the performance of the path planning functionalities.

# Installation steps

```bash
git clone https://github.com/AlexanderRex/warehouse_project.git
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
# Usage

Explain how to use the package, including launching the individual components and any necessary configuration steps.


Launch the cartographer for mapping:
```bash
ros2 launch cartographer_slam cartographer.launch.py
```
This command starts the Cartographer SLAM process to create a map of the warehouse environment using the cartographer package.
Launch the map server with a simulation map:

```bash
ros2 launch map_server map_server.launch.py use_sim_time:=true
```

This command launches the map server node, which publishes the generated map of the warehouse. The map file for the simulation environment is specified.
Launch the map server with a real environment map:

```bash
ros2 launch map_server map_server.launch.py use_sim_time:=true
```

Similar to the previous command, but this time it launches the map server with the map of the real warehouse environment.
Launch the localization server with a simulation map:

```bash
ros2 launch localization_server localization.launch.py use_sim_time:=true
```

This command starts the localization server using the simulation map. The server is responsible for the robot's self-localization within the map.
Launch the localization server with a real environment map:

```bash
ros2 launch localization_server localization.launch.py use_sim_time:=true
```
This command is used to launch the localization server with the map of the real environment.
Launch the path planner server:

```bash
ros2 launch path_planner_server pathplanner.launch.py
```

This command initiates the path planning process, enabling the robot to navigate autonomously in the warehouse.
